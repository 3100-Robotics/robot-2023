package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.driveTrainConstants;

public class drivetrain extends SubsystemBase{

    // motors
    private static WPI_TalonFX frontleftMotor = 
        new WPI_TalonFX(driveTrainConstants.frontLeftPort);
    private static WPI_TalonFX frontRightMotor = 
        new WPI_TalonFX(driveTrainConstants.frontRightPort);
    private static WPI_TalonFX backLeftMotor = 
        new WPI_TalonFX(driveTrainConstants.backLeftPort);
    private static WPI_TalonFX backRightMotor = 
        new WPI_TalonFX(driveTrainConstants.backRightPort);

    // drivetrain
    private static DifferentialDrive drive = new DifferentialDrive(frontleftMotor, frontRightMotor);

    // slew rate limiter
    private SlewRateLimiter speedLimiter = new SlewRateLimiter(driveTrainConstants.driveSlewRate);
    private SlewRateLimiter turnLimiter = new SlewRateLimiter(driveTrainConstants.turnSlewRate);

    // pid controller. named driveController in case I want to add a turn controller
    private PIDController driveController = new PIDController(
        driveTrainConstants.k_driveP, driveTrainConstants.k_driveI, driveTrainConstants.k_driveD);

    // navx gyro
    private static AHRS gyro = new AHRS(SPI.Port.kMXP);
    
    // slowmode for presision
    public Boolean slowmode = false;

    public drivetrain() {
        // deadband
        drive.setDeadband(0.05);
        // pid
        driveController.enableContinuousInput(-180, 180);
        driveController.setTolerance(driveTrainConstants.kDriveToleranceMeter,
            driveTrainConstants.kDriveRateToleranceMeterPerS);
        // config
        configureMotors();
        resetEncoders();
    }

    public void ToggleSlowMode() {
        // enable/disable the slow mode
        slowmode = !slowmode;
    }

    public void arcadeDrive(double speed, double rotation) {
        // arcade drive
        drive.arcadeDrive(speedLimiter.calculate(speed), rotation, false);
        // drive.arcadeDrive(speedLimiter.calculate(rotation), turnLimiter.calculate(rotation), false);
    }

    public void curvyDrive(double speed, double rotation, Boolean allowTurnInPlace) {
        // curvy drive will be odd because dosn't look like it sqares inputs
        drive.curvatureDrive(speed, rotation, false);
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        // tank drive
        drive.tankDrive(leftSpeed, rightSpeed, true);
    }

    public double getAverageEncoderRotation() {
        // average encoder distance
        return (frontleftMotor.getSelectedSensorPosition(0) + frontRightMotor.getSelectedSensorPosition(0))/2;
    }

    public void resetEncoders() {
        // reset encoders
        frontleftMotor.getSensorCollection().setIntegratedSensorPosition(0, 0);
        frontRightMotor.getSensorCollection().setIntegratedSensorPosition(0, 0);
    }

    public double getgyroz() {
        // get that pitch
        return gyro.getPitch();
    }

    public double getgyrox() {
        // get that yaw
        return gyro.getYaw();
    }

    public double getgyroy() {
        // get that roll
        return gyro.getRoll();
    }

    public void setSetpoint(double setpoint) {
        // set pid setpoint
        driveController.setSetpoint(setpoint);
    }

    public boolean atSetpoint() {
        // am I at pid setpoint?
        return driveController.atSetpoint();
    }

    public double driveCalculate(double measurement) {
        // calculate pid speed
        return driveController.calculate(measurement);
    }

    @Override
    public void periodic() {
        // useful info
        // SmartDashboard.putNumber("left speed", frontleftMotor.getSensorCollection().getIntegratedSensorVelocity());
        // SmartDashboard.putNumber("right speed", frontRightMotor.getSensorCollection().getIntegratedSensorVelocity());
        SmartDashboard.putData(drive);
        SmartDashboard.putData(gyro);
    }

    private void configureMotors() {
        // configs. fairly certain I need this var
        TalonFXConfiguration configs = new TalonFXConfiguration();

        // factory default
        frontleftMotor.configFactoryDefault();
        backLeftMotor.configFactoryDefault();
        frontRightMotor.configFactoryDefault();
        backRightMotor.configFactoryDefault();

        // safty good
        frontleftMotor.setSafetyEnabled(false);
        backLeftMotor.setSafetyEnabled(false);
        frontRightMotor.setSafetyEnabled(false);
        backRightMotor.setSafetyEnabled(false);

        // invert motors
        frontleftMotor.setInverted(false);
        backLeftMotor.setInverted(false);
        frontRightMotor.setInverted(true);
        backRightMotor.setInverted(true);

        // brake mode
        frontleftMotor.setNeutralMode(NeutralMode.Brake);
        backLeftMotor.setNeutralMode(NeutralMode.Brake);
        frontRightMotor.setNeutralMode(NeutralMode.Brake);
        backRightMotor.setNeutralMode(NeutralMode.Brake);

        // follow motors
        backLeftMotor.follow(frontleftMotor);
        backRightMotor.follow(frontRightMotor);

        // peak output for left
        frontleftMotor.configPeakOutputForward(1.0);
        frontleftMotor.configPeakOutputReverse(-1.0);

        // peak output for right
        frontRightMotor.configPeakOutputForward(1.0);
        frontRightMotor.configPeakOutputReverse(-1.0);

        // what sensor to use?
        frontleftMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        frontRightMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    }
}
