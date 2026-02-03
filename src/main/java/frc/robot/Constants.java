// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  private Constants() {
    // Prevent instantiation
  }
  
  // Enable tuning the PID and feedforward controller constants
  public static final boolean TUNING_MODE = true;
  public static final class DriveConstants {
    private DriveConstants() {
      // Prevent instantiation
    }

    // Motor controller IDs for drivetrain motors
    public static final int LEFT_LEADER_ID = 1;
    public static final int LEFT_FOLLOWER_ID = 2;
    public static final int RIGHT_LEADER_ID = 3;
    public static final int RIGHT_FOLLOWER_ID = 4;

    // Current limit for drivetrain motors. 60A is a reasonable maximum to reduce
    // likelihood of tripping breakers or damaging CIM motors
    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 60;
  }

  public static final class FuelConstants {
    private FuelConstants() {
      // Prevent instantiation
    }

    // Motor controller IDs for Fuel Mechanism motors
    public static final int FEEDER_MOTOR_ID = 16;
    public static final int LAUNCHER_MOTOR_ID = 15;
    public static final int INTAKE_MOTOR_ID = 14;

    // Current limit and nominal voltage for fuel mechanism motors.
    public static final int FEEDER_MOTOR_CURRENT_LIMIT = 60;
    public static final int LAUNCHER_MOTOR_CURRENT_LIMIT = 60;
    public static final int INTAKE_MOTOR_CURRENT_LIMIT = 60;

    // Voltage values for various fuel operations. These values may need to be tuned
    // based on exact robot construction.
    // See the Software Guide for tuning information
    public static final double INTAKING_FEEDER_VOLTAGE = -12;
    public static final double INTAKING_INTAKE_VOLTAGE = 3;
    public static final double LAUNCHING_FEEDER_VOLTAGE = 9;
    public static final double LAUNCHING_INTAKE_VOLTAGE = 4;
    public static final double SPIN_UP_FEEDER_VOLTAGE = -6;
    public static final double SPIN_UP_SECONDS = 1.0;
    public static final double RATE_LIMIT = 999.0; // volts per second

    // Constants tunable through TunableNumbers
    public static final double LAUNCHER_SPEED_RPM = 4800.0;
    public static final double LAUNCHER_KP_VOLTS_PER_RPM = 0.002;
    public static final double LAUNCHER_KS_VOLTS = 0.0;
    public static final double LAUNCHER_KV_VOLTS_PER_RPM = 0.0022;
    public static final double LAUNCHER_KA_VOLTS_PER_RPM2 = 0.0;

    /** Motor simulation constants. */
    public static final double POUND_IN2_TO_KG_METERS2 = Units.lbsToKilograms(1) 
                               * Math.pow(Units.inchesToMeters(1), 2);
    public static final double FEEDER_MOTOR_MOI_IN_LBS2 = 0.5;
    public static final double FEEDER_MOTOR_MOI_KG_METERS2 = FEEDER_MOTOR_MOI_IN_LBS2 * POUND_IN2_TO_KG_METERS2;
        public static final double LAUNCHER_MOTOR_MOI_IN_LBS2 = 5.0;
    public static final double LAUNCHER_MOTOR_MOI_KG_METERS2 = LAUNCHER_MOTOR_MOI_IN_LBS2 * POUND_IN2_TO_KG_METERS2;
    public static final double INTAKE_MOTOR_MOI_IN_LBS2 = 1.0;
    public static final double INTAKE_MOTOR_MOI_KG_METERS2 = INTAKE_MOTOR_MOI_IN_LBS2 * POUND_IN2_TO_KG_METERS2;
  }

  public static final class OperatorConstants {
    private OperatorConstants() {
      // Prevent instantiation
    }

    // Port constants for driver and operator controllers. These should match the
    // values in the Joystick tab of the Driver Station software
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    // This value is multiplied by the joystick value when driving the robot to
    // help avoid driving and turning too fast and being difficult to control
    public static final double DRIVE_SCALING = .35;
    public static final double ROTATION_SCALING = .4;
  }
}
