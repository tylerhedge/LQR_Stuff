// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a sample program to demonstrate how to use a state-space controller to control a
 * flywheel.
 */
public class Robot extends TimedRobot {
  //real stuff
  private static final int kLeadID = 9;
  private static final int kFollowerID = 11;
  private static final TalonFXInvertType kFollowerInvert = TalonFXInvertType.OpposeMaster;
  private TalonFX lead;
  private TalonFX follow;

  //sim stuff
  private static final int kMotorPort = 0;
  private static final int kEncoderAChannel = 0;
  private static final int kEncoderBChannel = 1;
  private static final int kJoystickPort = 0;
  private static final double kSpinupRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(500.0);
  private boolean spinningUp;

  private static final double kFlywheelMomentOfInertia = 0.00209; // kg * m^2

  // Reduction between motors and encoder, as output over input. If the flywheel spins slower than
  // the motors, this number should be greater than one.
  private static final double kFlywheelGearing = 60/48  ;

  // The plant holds a state-space model of our flywheel. This system has the following properties:
  //
  // States: [velocity], in radians per second.
  // Inputs (what we can "put in"): [voltage], in volts.
  // Outputs (what we can measure): [velocity], in radians per second.
  private final LinearSystem<N1, N1, N1> m_flywheelPlant =
      LinearSystemId.createFlywheelSystem(
          DCMotor.getFalcon500(2), kFlywheelMomentOfInertia, kFlywheelGearing);

  // The observer fuses our encoder data and voltage inputs to reject noise.
  private final KalmanFilter<N1, N1, N1> m_observer =
      new KalmanFilter<>(
          Nat.N1(),
          Nat.N1(),
          m_flywheelPlant,
          VecBuilder.fill(3.0), // How accurate we think our model is
          VecBuilder.fill(0.01), // How accurate we think our encoder
          // data is
          0.020);

  // A LQR uses feedback to create voltage commands.
  private final LinearQuadraticRegulator<N1, N1, N1> m_controller =
      new LinearQuadraticRegulator<>(
          m_flywheelPlant,
          VecBuilder.fill(20), // qelms. Velocity error tolerance, in radians per second. Decrease
          // this to more heavily penalize state excursion, or make the controller behave more
          // aggressively.
          VecBuilder.fill(6), // relms. Control effort (voltage) tolerance. Decrease this to more
          // heavily penalize control effort, or make the controller less aggressive. 12 is a good
          // starting point because that is the (approximate) maximum voltage of a battery.
          0.020); // Nominal time between loops. 0.020 for TimedRobot, but can be
  // lower if using notifiers.

  // The state-space loop combines a controller, observer, feedforward and plant for easy control.
  private final LinearSystemLoop<N1, N1, N1> m_loop =
      new LinearSystemLoop<>(m_flywheelPlant, m_controller, m_observer, 12, 0.020);

  // An encoder set up to measure flywheel velocity in radians per second.
  private final Encoder m_encoder = new Encoder(kEncoderAChannel, kEncoderBChannel);
  private final FlywheelSim m_flywheelSim = new FlywheelSim(m_flywheelPlant, DCMotor.getFalcon500(2), 2);
  private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);
  private final MotorController m_motor = new PWMSparkMax(kMotorPort);
  boolean shoot = false;

  // A joystick to read the trigger from.
  private final Joystick m_joystick = new Joystick(kJoystickPort);

  @Override
  public void robotInit() {
    // We go 2 pi radians per 4096 clicks.
    SmartDashboard.putBoolean("spinup", false);
    SmartDashboard.putNumber("target", 732);
    SmartDashboard.putBoolean("shoot", false);
    lead = new TalonFX(kLeadID);
    follow = new TalonFX(kFollowerID);
    follow.setInverted(kFollowerInvert);
    follow.follow(lead);

    lead.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40.0, 40.0, 1.0));
    follow.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40.0, 40.0, 1.0));


  }

  @Override
  public void teleopInit() {
    m_loop.reset(VecBuilder.fill(0));
  }

  @Override
  public void simulationInit(){
    m_loop.reset(VecBuilder.fill(m_encoderSim.getRate()));
  }

  @Override 
  public void simulationPeriodic(){
    m_encoderSim.setRate(m_flywheelSim.getAngularVelocityRadPerSec());
      // Sets the target speed of our flywheel. This is similar to setsting the setpoint of a
    // PID controller.
    if (SmartDashboard.getBoolean("spinup", false)) {
      // We just pressed the trigger, so let's set our next reference
      m_loop.setNextR(VecBuilder.fill(SmartDashboard.getNumber("target", 0)));
    } else {
      // We just released the trigger, so let's spin down
    }
    if (SmartDashboard.getBoolean("shoot", false) == true && shoot == false) m_encoderSim.setRate(m_encoderSim.getRate() - 100);
    shoot = SmartDashboard.getBoolean("shoot", false);
    // Correct our Kalman filter's state vector estimate with encoder data.
    m_loop.correct(VecBuilder.fill(m_encoderSim.getRate()));

    // Update our LQR to generate new voltage commands and use the voltages to predict the next
    // state with out Kalman filter.
    
    m_loop.predict(0.020);

    // Send the new calculated voltage to the motors.
    // voltage = duty cycle * battery voltage, so
    // duty cycle = voltage / battery voltage
    double nextVoltage = m_loop.getU(0);
    SmartDashboard.putNumber("nextv",  nextVoltage);
    if (!SmartDashboard.getBoolean("spinup", false)) nextVoltage = 0;
    m_flywheelSim.setInputVoltage(nextVoltage);
    m_flywheelSim.update(0.02);

    SmartDashboard.putNumber("error", m_loop.getError(0));
    SmartDashboard.putNumber("effort (current)", m_flywheelSim.getCurrentDrawAmps());
    SmartDashboard.putNumber("velocity", m_encoderSim.getRate());
  }

  @Override
  public void teleopPeriodic() {
    spinningUp = SmartDashboard.getBoolean("spinup", false);
    if (spinningUp) {
      // We just pressed the trigger, so let's set our next reference
      m_loop.setNextR(VecBuilder.fill(SmartDashboard.getNumber("target", 0))); 
    }
    else {
      m_loop.setNextR(VecBuilder.fill(0)); 
    }

    SmartDashboard.putNumber("controller error", m_loop.getError().get(0, 0));
    m_loop.correct(VecBuilder.fill(getEncoderRadsPerSec()));
    SmartDashboard.putNumber("encoder rads per sec", getEncoderRadsPerSec());
    m_loop.predict(0.020);
    double nextVoltage = m_loop.getU(0);
    System.out.println(nextVoltage);
    if (!spinningUp) nextVoltage = 0;
    lead.set(TalonFXControlMode.PercentOutput, nextVoltage/RobotController.getBatteryVoltage());



  }

  public double getEncoderRadsPerSec(){
    double temp; //motor ticks/100ms
    temp = lead.getSelectedSensorVelocity()/kFlywheelGearing; //"flywheel ticks" per 100 ms
    temp /=2048; //flywheel rotations per 100ms
    temp *= 10; //flywheel rotations per second
    temp *= (2*Math.PI); //flywheel radians per second
    return temp;
  }
}
