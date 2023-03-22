package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.driveTrainConstants;

public class Drive extends SubsystemBase{

    // motors
    private static final WPI_TalonFX frontLeftMotor =
        new WPI_TalonFX(driveTrainConstants.frontLeftPort);
    private static final WPI_TalonFX frontRightMotor =
        new WPI_TalonFX(driveTrainConstants.frontRightPort);
    private static final WPI_TalonFX backLeftMotor =
        new WPI_TalonFX(driveTrainConstants.backLeftPort);
    private static final WPI_TalonFX backRightMotor =
        new WPI_TalonFX(driveTrainConstants.backRightPort);

    // drivetrain
    private static final DifferentialDrive drive = new DifferentialDrive(frontLeftMotor, frontRightMotor);

    // slew rate limiter
    private final SlewRateLimiter speedLimiter = new SlewRateLimiter(driveTrainConstants.driveSlewRate);

    // pid controller. named driveController in case I want to add a turn controller
    private final PIDController driveController = new PIDController(
        driveTrainConstants.k_driveP, driveTrainConstants.k_driveI, driveTrainConstants.k_driveD);

    // navx gyro
    private static final AHRS gyro = new AHRS(SPI.Port.kMXP);
    
    private final DifferentialDriveOdometry odometry;

    public Drive() {
        // deadband
        drive.setDeadband(0.05);
        // pid
        driveController.enableContinuousInput(-180, 180);
        driveController.setTolerance(driveTrainConstants.kDriveToleranceMeter,
            driveTrainConstants.kDriveRateToleranceMeterPerS);

        
        // config
        configureMotors();
        resetEncoders();
        odometry =
            new DifferentialDriveOdometry(
                gyro.getRotation2d(), frontLeftMotor.getSelectedSensorPosition(), frontRightMotor.getSelectedSensorPosition());
    }

    public void arcadeDrive(double speed, double rotation) {
        // arcade drive
        drive.arcadeDrive(speedLimiter.calculate(speed), rotation, false);
        // drive.arcadeDrive(speedLimiter.calculate(rotation), turnLimiter.calculate(rotation), false);
    }

    public void curvyDrive(double speed, double rotation, Boolean allowTurnInPlace) {
        drive.curvatureDrive(speed, rotation, allowTurnInPlace);
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        // tank drive
        drive.tankDrive(leftSpeed, rightSpeed, true);
    }

    public double getAverageEncoderRotation() {
        // average encoder distance
        return (frontLeftMotor.getSelectedSensorPosition(0) + frontRightMotor.getSelectedSensorPosition(0))/2;
    }

    public void resetEncoders() {
        // reset encoders
        frontLeftMotor.getSensorCollection().setIntegratedSensorPosition(0, 0);
        frontRightMotor.getSensorCollection().setIntegratedSensorPosition(0, 0);
    }

    public double getGyroZ() {
        // get that pitch
        return gyro.getPitch();
    }

    public double getGyroX() {
        // get that yaw
        return gyro.getYaw();
    }

    public double getGyroY() {
        // get that roll
        return gyro.getRoll();
    }

    public void setSetpoint(double setpoint) {
        // set pid setpoint
        System.out.println(setpoint);
        driveController.setSetpoint(setpoint);
    }

    public boolean atSetpoint() {
        // am I at pid setpoint?
        System.out.println(driveController.atSetpoint());
        return driveController.atSetpoint();
    }

    public double driveCalculate(double measurement) {
        // calculate pid speed
        return driveController.calculate(measurement);
    }

    public void setBrakeMode(NeutralMode mode) {
        frontLeftMotor.setNeutralMode(mode);
        backLeftMotor.setNeutralMode(mode);
        frontRightMotor.setNeutralMode(mode);
        backRightMotor.setNeutralMode(mode);
    }

    @Override
    public void periodic() {
        // useful info
        SmartDashboard.putData(drive);
        SmartDashboard.putData(gyro);
    }

    private void configureMotors() {
        // configs. fairly certain I need this var
        TalonFXConfiguration configs = new TalonFXConfiguration();

        // factory default
        frontLeftMotor.configFactoryDefault();
        backLeftMotor.configFactoryDefault();
        frontRightMotor.configFactoryDefault();
        backRightMotor.configFactoryDefault();

        // safety good
        frontLeftMotor.setSafetyEnabled(false);
        backLeftMotor.setSafetyEnabled(false);
        frontRightMotor.setSafetyEnabled(false);
        backRightMotor.setSafetyEnabled(false);

        // invert motors
        frontLeftMotor.setInverted(true);
        backLeftMotor.setInverted(true);
        frontRightMotor.setInverted(false);
        backRightMotor.setInverted(false);

        // brake mode
        frontLeftMotor.setNeutralMode(NeutralMode.Brake);
        backLeftMotor.setNeutralMode(NeutralMode.Brake);
        frontRightMotor.setNeutralMode(NeutralMode.Brake);
        backRightMotor.setNeutralMode(NeutralMode.Brake);

        // follow motors
        backLeftMotor.follow(frontLeftMotor);
        backRightMotor.follow(frontRightMotor);

        // peak output for left
        frontLeftMotor.configPeakOutputForward(1.0);
        frontLeftMotor.configPeakOutputReverse(-1.0);

        // peak output for right
        frontRightMotor.configPeakOutputForward(1.0);
        frontRightMotor.configPeakOutputReverse(-1.0);

        // what sensor to use?
        frontLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        frontRightMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    }
}
