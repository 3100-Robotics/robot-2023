// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kCoDriverControllerPort = 1;
  }

  public static class driveTrainConstants {
    public static final int frontLeftPort = 1;
    public static final int frontRightPort = 2;
    public static final int backLeftPort = 3;
    public static final int backRightPort = 4;

    public final static double kSensorUnitsPerRotation = 2048;
    private final static double kGearReduction = 8.68;
    private final static double driveWheelRadiusMeters = 0.0508;

    public final static double kp = 1;
    public final static double ki = 1;
    public final static double kd = 1;
    public final static double kDriveToleranceMeter = 0.1;
    public final static double kDriveToleranceDeg = 5;
    public final static double kDriveRateToleranceMeterPerS = 0.2;

    public static double encoderScale = (1 / kGearReduction) * (kSensorUnitsPerRotation)
        * (2 * Math.PI * driveWheelRadiusMeters);
  }

  public static class ArmMotorConstants{
    public static final int armMotor1 = 5;
    public static final int armMotor2 = 6;

    public final static double kp = 0;
    public final static double ki = 0;
    public final static double kd = 0;

    public final static double kArmToleranceMeter = 0.1;
    public final static double kArmRateToleranceMeterPerS = 0.2;

    public final static double kSensorUnitsPerRotation = 42;
    private final static double kGearReduction = 1;
    public final static double tick2Feet = (1/kGearReduction) * (1/kSensorUnitsPerRotation);
    public final static double lvl1 = 1;
    public final static double lvl2 = 2;
    public final static double lvl3 = 3;
  }

  public static class ElevatorConstants{
    public static final int elevatorMotor1 = 7;
    public static final int elevatorMotor2 = 8;

    public final static double kp = 1;
    public final static double ki = 1;
    public final static double kd = 1;

    public final static double kSensorUnitsPerRotation = 42;
    private final static double kGearReduction = 1;
    private final static double tick2Feet = (1/kGearReduction) * (1/kSensorUnitsPerRotation);

    // setpoints in feet
    public final static double lvl1 = 1;
    public final static double lvl2 = 2;
    public final static double lvl3 = 3;
  }

  public static class endAffectorConstants{
    public static final int leftMotor = 9;
    public static final int rightMotor = 10;

    public final static double kp = 1;
    public final static double ki = 1;
    public final static double kd = 1;

    public final static double kSensorUnitsPerRotation = 42;
    private final static double kGearReduction = 1;
    private final static double tick2Feet = (1/kGearReduction) * (1/kSensorUnitsPerRotation);

    public final static double kAffectorToleranceMeter = 0.1;
    public final static double kAffectorRateToleranceMeterPerS = 0.2;
  }

  public static final class IOConstants {
    // === XBOX CHANNELS === //
    // AXES
    public static final int leftXAxisChannel = 0;
    public static final int leftYAxisChannel = 1;
    public static final int leftTriggerChannel = 2;
    public static final int rightTriggerChannel = 3;
    public static final int rightXAxisChannel = 4;
    public static final int rightYAxisChannel = 5;

    // BUTTONS
    public static final int aButtonChannel = 1;
    public static final int bButtonChannel = 2;
    public static final int xButtonChannel = 3;
    public static final int yButtonChannel = 4;

    public static final int leftBumperChannel = 5;
    public static final int rightBumperChannel = 6;

    public static final int backButtonChannel = 7;
    public static final int startButtonChannel = 8;

    public static final int POVU = 0;
    public static final int POVR = 90;
    public static final int POVD = 180;
    public static final int POVL = 270;
 }

}
