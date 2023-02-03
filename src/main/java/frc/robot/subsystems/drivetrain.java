package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.driveTrainConstants;

public class drivetrain extends SubsystemBase{

    private static WPI_TalonFX frontleftMotor = 
        new WPI_TalonFX(driveTrainConstants.frontLeftPort);
    private static WPI_TalonFX frontRightMotor = 
        new WPI_TalonFX(driveTrainConstants.frontRightPort);
    private static WPI_TalonFX backLeftMotor = 
        new WPI_TalonFX(driveTrainConstants.backLeftPort);
    private static WPI_TalonFX backRightMotor = 
        new WPI_TalonFX(driveTrainConstants.backRightPort);

    private static DifferentialDrive drive = new DifferentialDrive(frontleftMotor, frontRightMotor);

    private static AHRS gyro = new AHRS(SPI.Port.kMXP);

    public drivetrain() {
        configureMotors();
        resetEncoders();
    }

    public void arcadeDrive(double speed, double rotation) {
        drive.arcadeDrive(speed, rotation);
    }

    public double getAverageEncoderRotation() {
        return (frontleftMotor.getSelectedSensorPosition(0) + frontRightMotor.getSelectedSensorPosition(0))/2;
    }

    public void resetEncoders() {
        frontleftMotor.getSensorCollection().setIntegratedSensorPosition(0, 0);
        frontRightMotor.getSensorCollection().setIntegratedSensorPosition(0, 0);
    }

    public double getgyroz() {
        return gyro.getRawGyroZ();
    }

    public double getgyrox() {
        return gyro.getRawGyroX();
    }

    public double getgyroy() {
        return gyro.getRawGyroY();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("left speed", frontleftMotor.getSensorCollection().getIntegratedSensorVelocity());
        SmartDashboard.putNumber("right speed", frontRightMotor.getSensorCollection().getIntegratedSensorVelocity());
    }

    // public double getAverageEncoderVelocity() {} to be added later?

    private void configureMotors() {

        System.out.println("configuration");

        frontleftMotor.configFactoryDefault();
        frontRightMotor.configFactoryDefault();
        backLeftMotor.configFactoryDefault();
        backRightMotor.configFactoryDefault();

        System.out.println("factory default");

        frontleftMotor.setNeutralMode(NeutralMode.Brake);
        frontRightMotor.setNeutralMode(NeutralMode.Brake);
        backLeftMotor.setNeutralMode(NeutralMode.Brake);
        backRightMotor.setNeutralMode(NeutralMode.Brake);

        System.out.println("brake mode");

        frontleftMotor.setInverted(false);
        frontRightMotor.setInverted(false);
        backLeftMotor.setInverted(true);
        backRightMotor.setInverted(true);

        System.out.println("inversion");

        frontleftMotor.setSafetyEnabled(true);
        frontRightMotor.setSafetyEnabled(true);
        backLeftMotor.setSafetyEnabled(true);
        backRightMotor.setSafetyEnabled(true);

        System.out.println("safety");

        backLeftMotor.follow(frontleftMotor);
        backRightMotor.follow(frontRightMotor);

        System.out.println("following");

        frontleftMotor.configPeakOutputForward(1.0);
        frontRightMotor.configPeakOutputForward(1.0);

        frontleftMotor.configPeakOutputReverse(-1.0);
        frontRightMotor.configPeakOutputReverse(-1.0);

        System.out.println("max outputs");

        frontleftMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        frontRightMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        System.out.println("fedback device");
    }
    
}
