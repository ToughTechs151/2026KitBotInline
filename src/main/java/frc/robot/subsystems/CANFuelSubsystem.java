// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;

import static frc.robot.Constants.FuelConstants.*;

public class CANFuelSubsystem extends SubsystemBase {
  private final SparkMax feederRoller;
  private final SparkMax intakeLauncherRoller;
  private final RelativeEncoder feederEncoder;
  private final RelativeEncoder intakeLauncherEncoder;

  // Simulation objects
  private final SparkMaxSim feederSparkSim;
  private final SparkMaxSim intakeLauncherSparkSim;
  private final DCMotor motorGearbox = DCMotor.getNEO(1);
  private final LinearSystem<N2, N1, N2> feederPlant = LinearSystemId.createDCMotorSystem(motorGearbox,
      FEEDER_MOTOR_MOI_KG_METERS2, 1);
  private final DCMotorSim feederMotorSim = new DCMotorSim(feederPlant, motorGearbox);
  private final LinearSystem<N2, N1, N2> intakeLauncherPlant = LinearSystemId.createDCMotorSystem(motorGearbox,
      LAUNCHER_MOTOR_MOI_KG_METERS2, 1);
  private final DCMotorSim intakeLauncherMotorSim = new DCMotorSim(intakeLauncherPlant, motorGearbox);

  private static final String INTAKING_FEEDER_ROLLER_KEY = "Intaking feeder roller value";
  private static final String INTAKING_INTAKE_ROLLER_KEY = "Intaking intake roller value";
  private static final String LAUNCHING_FEEDER_ROLLER_KEY = "Launching feeder roller value";
  private static final String LAUNCHING_LAUNCHER_ROLLER_KEY = "Launching launcher roller value";
  private static final String SPINUP_FEEDER_ROLLER_KEY = "Spin-up feeder roller value";
  private SlewRateLimiter limiter;
  private double feederGoal = 0.0;
  private double intakeLauncherGoal = 0.0;

  /** Creates a new CANBallSubsystem. */
  public CANFuelSubsystem() {
    limiter = new SlewRateLimiter(RATE_LIMIT);
    // create brushed motors for each of the motors on the launcher mechanism
    intakeLauncherRoller = new SparkMax(INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
    feederRoller = new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushless);
    feederEncoder = feederRoller.getEncoder();
    intakeLauncherEncoder = intakeLauncherRoller.getEncoder();

    intakeLauncherSparkSim = new SparkMaxSim(intakeLauncherRoller, motorGearbox);
    feederSparkSim = new SparkMaxSim(feederRoller, motorGearbox);

    // put default values for various fuel operations onto the dashboard
    // all methods in this subsystem pull their values from the dashboard to allow
    // you to tune the values easily, and then replace the values in Constants.java
    // with your new values. For more information, see the Software Guide.
    SmartDashboard.putNumber(INTAKING_FEEDER_ROLLER_KEY, INTAKING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber(INTAKING_INTAKE_ROLLER_KEY, INTAKING_INTAKE_VOLTAGE);
    SmartDashboard.putNumber(LAUNCHING_FEEDER_ROLLER_KEY, LAUNCHING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber(LAUNCHING_LAUNCHER_ROLLER_KEY, LAUNCHING_LAUNCHER_VOLTAGE);
    SmartDashboard.putNumber(SPINUP_FEEDER_ROLLER_KEY, SPIN_UP_FEEDER_VOLTAGE);

    // create the configuration for the feeder roller, set a current limit and apply
    // the config to the controller
    SparkMaxConfig feederConfig = new SparkMaxConfig();
    feederConfig.smartCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT);
    feederRoller.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // create the configuration for the launcher roller, set a current limit, set
    // the motor to inverted so that positive values are used for both intaking and
    // launching, and apply the config to the controller
    SparkMaxConfig launcherConfig = new SparkMaxConfig();
    launcherConfig.inverted(true);
    launcherConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
    intakeLauncherRoller.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // A method to set the rollers to values for intaking
  public void intake() {
    feederGoal = SmartDashboard.getNumber(INTAKING_FEEDER_ROLLER_KEY, INTAKING_FEEDER_VOLTAGE);
    intakeLauncherGoal = SmartDashboard.getNumber(INTAKING_INTAKE_ROLLER_KEY, INTAKING_INTAKE_VOLTAGE);
  }

  // A method to set the rollers to values for ejecting fuel out the intake. Uses
  // the same values as intaking, but in the opposite direction.
  public void eject() {
    feederGoal = SmartDashboard.getNumber(INTAKING_FEEDER_ROLLER_KEY, INTAKING_FEEDER_VOLTAGE);
    intakeLauncherGoal = -1 * SmartDashboard.getNumber(INTAKING_INTAKE_ROLLER_KEY, INTAKING_INTAKE_VOLTAGE);
  }

  // A method to set the rollers to values for launching.
  public void launch() {
    feederGoal = SmartDashboard.getNumber(LAUNCHING_FEEDER_ROLLER_KEY, LAUNCHING_FEEDER_VOLTAGE);
    intakeLauncherGoal = SmartDashboard.getNumber(LAUNCHING_LAUNCHER_ROLLER_KEY, LAUNCHING_LAUNCHER_VOLTAGE);
  }

  // A method to stop the rollers
  public void stop() {
    feederGoal = 0.0;
    intakeLauncherGoal = 0.0;
  }

  // A method to spin up the launcher roller while spinning the feeder roller to
  public void spinUp() {
    feederGoal = SmartDashboard.getNumber(SPINUP_FEEDER_ROLLER_KEY, SPIN_UP_FEEDER_VOLTAGE);
    intakeLauncherGoal = SmartDashboard.getNumber(LAUNCHING_LAUNCHER_ROLLER_KEY, LAUNCHING_LAUNCHER_VOLTAGE);
  }

  // A command factory to turn the spinUp method into a command that requires this
  // subsystem
  public Command spinUpCommand() {
    return this.run(this::spinUp);
  }

  // A command factory to turn the launch method into a command that requires this
  // subsystem
  public Command launchCommand() {
    return this.run(this::launch);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Use the slew rate limiter to ramp the feeder voltage to avoid sudden changes
    double feederVoltage = limiter.calculate(feederGoal);
    feederRoller.setVoltage(feederVoltage);
    intakeLauncherRoller.setVoltage(intakeLauncherGoal);

    // Simulate the roller motors in simulation mode
    if (RobotBase.isSimulation()) {
      feederMotorSim.setInput(feederVoltage);
      feederMotorSim.update(0.020);
      feederSparkSim.iterate(feederMotorSim.getAngularVelocityRPM(), 12.0, 0.02);
      
      intakeLauncherMotorSim.setInput(intakeLauncherGoal);
      intakeLauncherMotorSim.update(0.020);
      intakeLauncherSparkSim.iterate(intakeLauncherMotorSim.getAngularVelocityRPM(), 12.0, 0.02);
    }

    // Update SmartDashboard values for monitoring
    SmartDashboard.putNumber("Feeder Goal", feederGoal);
    SmartDashboard.putNumber("Feeder Set Voltage", feederVoltage);
    SmartDashboard.putNumber("Launcher Goal", intakeLauncherGoal);

    SmartDashboard.putNumber("LauncherCurrent", intakeLauncherRoller.getOutputCurrent());
    SmartDashboard.putNumber("LauncherVoltage", intakeLauncherRoller.getAppliedOutput()
        * intakeLauncherRoller.getBusVoltage());
    SmartDashboard.putNumber("LauncherVelocity", intakeLauncherEncoder.getVelocity());

    SmartDashboard.putNumber("FeederCurrent", feederRoller.getOutputCurrent());
    SmartDashboard.putNumber("FeederVoltage", feederRoller.getAppliedOutput()
        * feederRoller.getBusVoltage());
    SmartDashboard.putNumber("FeederVelocity", feederEncoder.getVelocity());
  }
}
