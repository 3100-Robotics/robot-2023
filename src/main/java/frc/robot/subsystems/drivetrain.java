package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
// import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.driveTrainConstants;;

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

    //TODO make this work once the gyro lib has updated
    // private static Gyro gyro = new AHRS(SPI.Port.kMXP);

    public drivetrain() {
        configureMotors();
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

    // public double getgyrox() {
    //     return gyro.
    // }

    // public double getAverageEncoderVelocity() {} to be added later?

    private void configureMotors() {

        frontleftMotor.configFactoryDefault();
        frontRightMotor.configFactoryDefault();
        backLeftMotor.configFactoryDefault();
        backRightMotor.configFactoryDefault();

        frontleftMotor.setNeutralMode(NeutralMode.Brake);
        frontRightMotor.setNeutralMode(NeutralMode.Brake);
        backLeftMotor.setNeutralMode(NeutralMode.Brake);
        backRightMotor.setNeutralMode(NeutralMode.Brake);

        frontleftMotor.setInverted(false);
        frontRightMotor.setInverted(false);
        backLeftMotor.setInverted(true);
        backRightMotor.setInverted(true);

        frontleftMotor.setSafetyEnabled(true);
        frontRightMotor.setSafetyEnabled(true);
        backLeftMotor.setSafetyEnabled(true);
        backRightMotor.setSafetyEnabled(true);

        backLeftMotor.follow(frontleftMotor);
        backRightMotor.follow(frontRightMotor);

        frontleftMotor.configPeakOutputForward(1.0);
        frontRightMotor.configPeakOutputForward(1.0);

        frontleftMotor.configPeakOutputReverse(-1.0);
        frontRightMotor.configPeakOutputReverse(-1.0);

        frontleftMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        frontRightMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    }
    
}
