package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmMotorConstants;

public class Arm extends SubsystemBase {
    private static final double ENCODER_TICKS_TO_METERS = 1.0; // FIXME: Calculate with REV Hardware Client and real arm

    // motor
    private final CANSparkMax armMotor = new CANSparkMax(ArmMotorConstants.armMotor, MotorType.kBrushless);

    private final SlewRateLimiter limiter = new SlewRateLimiter(ArmMotorConstants.slewRate);
    private final RelativeEncoder internalEncoder = armMotor.getEncoder();
    // pid
    public PIDController controller;
    int setpointNum;

    private double targetExtension;

    public Arm() {
        // pid config
        setpointNum = 1;
        controller = new PIDController(ArmMotorConstants.kp, ArmMotorConstants.ki, ArmMotorConstants.kd);
//        controller.setSetpoint(ArmMotorConstants.armLevels[setpointNum - 1]);

        // encoder.setPositionConversionFactor(ArmMotorConstants.encoderPosFactor);

        armMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);
        armMotor.setSoftLimit(SoftLimitDirection.kForward, 58);
        armMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        armMotor.enableSoftLimit(SoftLimitDirection.kForward, false);

        // motor config
        armMotor.setInverted(false);
        armMotor.setIdleMode(IdleMode.kBrake);
    }

    public void setTargetExtension(double extensionMeters) {
        targetExtension = extensionMeters;
    }

    public double getExtensionMeters() {
        if (RobotBase.isSimulation()) { return targetExtension; } // Feed back demand if in simulation mode
        return internalEncoder.getPosition() * ENCODER_TICKS_TO_METERS;
    }

    public boolean atSetpoint() {
        // am I at setpoint
        return controller.atSetpoint();
    }

    @Override
    public void periodic() {
        // Set the motor output
        armMotor.set(
            controller.calculate(
                getExtensionMeters(),
                targetExtension
            )
        );

        SmartDashboard.putNumber("arm pos", GetEncoderRotation());
//        SmartDashboard.putBoolean("at setpoint", atSetpoint());
    }

    @Deprecated
    public void Run(double speed) {
        // run the motor
        armMotor.set(limiter.calculate(speed));
    }

    public void Stop(){
        // stop the motor
        armMotor.stopMotor();
    }

    @Deprecated
    public double GetEncoderRotation(){
        // get the encoder rot
        return internalEncoder.getPosition();
    }
}
