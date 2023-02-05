package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
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
    
    public Boolean slowmode;

    public drivetrain() {
        configureMotors();
        resetEncoders();
    }

    public void ToggleSlowMode() {
        slowmode = !slowmode;
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
        return gyro.getPitch();
    }

    public double getgyrox() {
        return gyro.getYaw();
    }

    public double getgyroy() {
        return gyro.getRoll();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("left speed", frontleftMotor.getSensorCollection().getIntegratedSensorVelocity());
        SmartDashboard.putNumber("right speed", frontRightMotor.getSensorCollection().getIntegratedSensorVelocity());
    }

    // public double getAverageEncoderVelocity() {} to be added later?

    private void configureMotors() {

        TalonFXConfiguration configs = new TalonFXConfiguration();

    frontleftMotor.configFactoryDefault();
    backLeftMotor.configFactoryDefault();
    frontRightMotor.configFactoryDefault();
    backRightMotor.configFactoryDefault();

    frontleftMotor.setSafetyEnabled(false);
    backLeftMotor.setSafetyEnabled(false);
    frontRightMotor.setSafetyEnabled(false);
    backRightMotor.setSafetyEnabled(false);
    

    // _leftConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
    // _rightConfig.remoteFilter0.remoteSensorDeviceID = frontLeft.getDeviceID(); // Device ID of Remote Source
    // _rightConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.TalonFX_SelectedSensor; // Remote Source Type

    // setRobotDistanceConfigs(_rightInvert, _rightConfig);
    // configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;

    // frontLeft.configAllSettings(configs);
    // frontRight.configAllSettings(configs);

    // Determines which motors will be inverted

    frontleftMotor.setInverted(false);
    backLeftMotor.setInverted(false);
    frontRightMotor.setInverted(true);
    backRightMotor.setInverted(true);

    // Sets the motors to brake mode
    frontleftMotor.setNeutralMode(NeutralMode.Brake);
    backLeftMotor.setNeutralMode(NeutralMode.Brake);
    frontRightMotor.setNeutralMode(NeutralMode.Brake);
    backRightMotor.setNeutralMode(NeutralMode.Brake);

    backLeftMotor.follow(frontleftMotor);
    backRightMotor.follow(frontRightMotor);

    frontleftMotor.configPeakOutputForward(1.0);
    frontleftMotor.configPeakOutputReverse(-1.0);

    frontRightMotor.configPeakOutputForward(1.0);
    frontRightMotor.configPeakOutputReverse(-1.0);

    frontleftMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    frontRightMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    // _rightConfig.slot2.kF = Constants.kGains_Velocit.kF;
    // _rightConfig.slot2.kP = Constants.kGains_Velocit.kP;
    // _rightConfig.slot2.kI = Constants.kGains_Velocit.kI;
    // _rightConfig.slot2.kD = Constants.kGains_Velocit.kD;
    // _rightConfig.slot2.integralZone = Constants.kGains_Velocit.kIzone;
    // _rightConfig.slot2.closedLoopPeakOutput = Constants.kGains_Velocit.kPeakOutput;

    // /* Config the neutral deadband. */
    // _leftConfig.neutralDeadband = Constants.kNeutralDeadband;
    // _rightConfig.neutralDeadband = Constants.kNeutralDeadband;

    // int closedLoopTimeMs = 1;
    // _rightConfig.slot0.closedLoopPeriod = closedLoopTimeMs;
    // _rightConfig.slot1.closedLoopPeriod = closedLoopTimeMs;
    // _rightConfig.slot2.closedLoopPeriod = closedLoopTimeMs;
    // _rightConfig.slot3.closedLoopPeriod = closedLoopTimeMs;

    // /* Motion Magic Configs */
    // _rightConfig.motionAcceleration = 2000; // (distance units per 100 ms) per second
    // _rightConfig.motionCruiseVelocity = 2000; // distance units per 100 ms

    // /* APPLY the config settings */
    // frontLeft.configAllSettings(_leftConfig);
    // frontRight.configAllSettings(_rightConfig);

    // /* Set status frame periods to ensure we don't have stale data */
    // frontRight.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, Constants.kTimeoutMs);
    // frontRight.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, Constants.kTimeoutMs);
    // frontLeft.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, Constants.kTimeoutMs);


  

    //leftMotorLeader.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, currentLimit, currentThreshold, currentThresholdTime));
    //rightMotorLeader.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, currentLimit, currentThreshold, currentThresholdTime));

    //Might break
    // frontLeft.setSensorPhase(true);
    // frontRight.setSensorPhase(true);

    // frontLeft.setSelectedSensorPosition(0);
    // frontRight.setSelectedSensorPosition(0);

    // Add PID constants
    // frontLeft.config_kP(0, 0);
    // frontLeft.config_kI(0, 0);
    // frontLeft.config_kD(0, 0);
    // frontLeft.config_kF(0, 0);
    // // leftMotorLeader.configMaxIntegralAccumulator(0, 400);

    // frontRight.config_kP(0, 0);
    // frontRight.config_kI(0, 0);
    // frontRight.config_kD(0, 0);
    // frontRight.config_kF(0, 0);
    // // rightMotorLeader.configMaxIntegralAccumulator(0, 400);
 
    // frontLeft.setIntegralAccumulator(0);
    // frontRight.setIntegralAccumulator(0);
    }
    
}
