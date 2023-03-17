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
    // porst for controllers
    public static final int kDriverControllerPort = 0;
    public static final int kCoDriverControllerPort = 1;
  }

  public static class driveTrainConstants {
    // ports for motors
    public static final int frontLeftPort = 1;
    public static final int frontRightPort = 2;
    public static final int backLeftPort = 3;
    public static final int backRightPort = 4;

    // ballencing pid
    public final static double k_balenceP = 0.01215;
    public final static double k_balenceI = 0;
    public final static double k_balenceD = 0;

    // driving pid
    public final static double k_driveP = 0.001;
    public final static double k_driveI = 0;
    public final static double k_driveD = 0;

    // slew rate constant
    public final static double driveSlewRate = 5;
    public final static double turnSlewRate = 5;

    // pid tollerances
    public final static double kDriveToleranceMeter = 0.1;
    public final static double kDriveToleranceDeg = 5;
    public final static double kDriveRateToleranceMeterPerS = 0.2;

    // gear reduction math
    public final static int kSensorUnitsPerRotation = 2048;
    private final static double kGearReduction = 5.51;
    private final static double kWheelRadious = 3;
    private final static double kRads2inches = 2 * Math.PI * kWheelRadious;
    public final static double feet2tick = (kGearReduction) * (kSensorUnitsPerRotation) / (kRads2inches / 12);
    public final static double tick2feet = 1/feet2tick;
  }

  public static class ElevatorConstants{
    // motor ports
    public static final int LeftElevatorMotor = 6;
    public static final int RightElevatorMotor = 7;

    public static final double slewRate = 300;

    // pid
    public final static double kp = 0;
    public final static double ki = 0;
    public final static double kd = 0;

    // gear ratio math
    public final static int kSensorUnitsPerRotation = 42;
    private final static double kGearReduction = 20;
    private final static double kWinchRadious = 1.72/2;
    public final static double kInches2Rots = 2 * Math.PI * kWinchRadious / kGearReduction;
    public final static double kRads2inches = 1/kInches2Rots;
    public final static double kcircumperence = 2 * Math.PI * kWinchRadious;
    public final static double feet2tick = (kGearReduction) * (kSensorUnitsPerRotation) / (kcircumperence / 12);
    public final static double tick2Feet = 1/feet2tick;

    // fancy level stuff isn't used right now
    public static double[] elevatorLevels = {1.0, 2.0, 3.0, 4.0, 5.0}; {
      for (int i = 0; i < elevatorLevels.length; ++i) {
        elevatorLevels[i] *= tick2Feet;
      }
    }
  }

  public static class ArmMotorConstants{
    // motor port
    public static final int armMotor = 5;

    public static final double slewRate = 300;

    // pid
    public final static double kp = 0;
    public final static double ki = 0;
    public final static double kd = 0;

    // pid tolerences
    public final static double kArmToleranceMeter = 0.1;
    public final static double kArmRateToleranceMeterPerS = 0.2;

    // gear ratio math
    public final static int kSensorUnitsPerRotation = 42;
    private final static double kGearReduction = 20;
    private final static double kWinchRadious = 1.9/2;
    public final static double kInches2Rots = 2 * Math.PI * kWinchRadious / kGearReduction;
    public final static double kRads2inches = 1/kInches2Rots;
    public final static double kcircumperence = 2 * Math.PI * kWinchRadious;
    public final static double feet2tick = (kGearReduction) * (kSensorUnitsPerRotation) / (kcircumperence / 12);
    public final static double tick2Feet = 1/feet2tick;

    public final static double doubleEcoderPosFactor = kRads2inches * 60;
    public final static float softLimitRots = (float)doubleEcoderPosFactor;

    // fancy level stuff isn't used right now
    public static double[] armLevels = {1, 2, 3}; {
      for (int i = 0; i < armLevels.length; ++i) {
        armLevels[i] *= tick2Feet;
      }
    }
  }

  public static class endAffectorConstants{
    // motor ports
    public static final int leftMotor = 8;
    public static final int rightMotor = 9;

    // pid
    public final static double kp = 0;
    public final static double ki = 0;
    public final static double kd = 0;

    // gear ratio math
    public final static int kSensorUnitsPerRotation = 42;
    private final static double kGearReduction = 25;
    private final static double kWinchRadious = 1.29/2;
    public final static double kInches2Rots = 2 * Math.PI * kWinchRadious / kGearReduction;
    public final static double kRots2inches = 1/kInches2Rots;
    public final static double kcircumperence = 2 * Math.PI * kWinchRadious;
    public final static double feet2tick = (kGearReduction) * (kSensorUnitsPerRotation) / (kcircumperence / 12);
    public final static double tick2Feet = 1/feet2tick;

    public final static double doublesoftLimitRots = 20;
    public final static float rightSoftLimitRots = (float)doublesoftLimitRots;
    public final static float leftSoftLimitRots = (float)doublesoftLimitRots;

    // pid tolerances
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
