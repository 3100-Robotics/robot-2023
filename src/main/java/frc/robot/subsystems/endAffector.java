package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.endAffectorConstants;

public class endAffector extends SubsystemBase{
    // motors
    private CANSparkMax leftAffector = new CANSparkMax(endAffectorConstants.leftMotor, MotorType.kBrushless);
    private CANSparkMax rightAffector = new CANSparkMax(endAffectorConstants.rightMotor, MotorType.kBrushless);

    // encoders for motors change to absolute when they exist
    private AbsoluteEncoder LeftEncoder = leftAffector.getAbsoluteEncoder(Type.kDutyCycle);
    private AbsoluteEncoder righEncoder = rightAffector.getAbsoluteEncoder(Type.kDutyCycle);
    // private RelativeEncoder LefEncoder = leftAffector.getEncoder();
    // private RelativeEncoder RighEncoder = rightAffector.getEncoder();

    // are the joystick controlls locked?
    public Boolean endAffectorLock = false;

    public endAffector() {
        // brake mode!
        leftAffector.setIdleMode(IdleMode.kBrake);
        rightAffector.setIdleMode(IdleMode.kBrake);
        
        LeftEncoder.setPositionConversionFactor(endAffectorConstants.encoderPosFactor);
        righEncoder.setPositionConversionFactor(endAffectorConstants.encoderPosFactor);

        leftAffector.setSoftLimit(SoftLimitDirection.kForward, endAffectorConstants.encoderPosFactor);
        leftAffector.setSoftLimit(SoftLimitDirection.kReverse, 0);
        leftAffector.enableSoftLimit(SoftLimitDirection.kReverse, true);
        leftAffector.enableSoftLimit(SoftLimitDirection.kForward, true);

        rightAffector.setSoftLimit(SoftLimitDirection.kReverse, endAffectorConstants.encoderPosFactor);
        rightAffector.setSoftLimit(SoftLimitDirection.kForward, 0);
        rightAffector.enableSoftLimit(SoftLimitDirection.kReverse, true);
        rightAffector.enableSoftLimit(SoftLimitDirection.kForward, true);
    }

    @Override
    public void periodic() {
        // useful details
        SmartDashboard.putBoolean("end affector locked", endAffectorLock);
    }

    public void toggleEndAffectorLock() {
        // lock/unlock the joystick controlls
        endAffectorLock = !endAffectorLock;
    }

    public void runLeft(double speed) {
        // run the left claw
        leftAffector.set(speed);
    }

    public void runRight(double speed) {
        // run the right claw
        rightAffector.set(speed);
    }

    public void stopLeft() {
        // stop the left claw
        leftAffector.stopMotor();
    }

    public void stopRight() {
        // stop the right claw
        rightAffector.stopMotor();
    }

    
    public void runBoth(double speed) {
        // run both claws positive number = to the left
        leftAffector.set(speed);
        rightAffector.set(-speed);
    }

    public void stopBoth() {
        // stop both claws
        leftAffector.stopMotor();
        rightAffector.stopMotor();
    }

    public double getLeftEncoder() {
        // get the left encoder pos
        return LeftEncoder.getPosition();
    }

    public double getRightEncoder() {
        // get the right encoder pos
        return righEncoder.getPosition();
    }

    public double getCenterPos() {
        // get the center point between the claws
        return (LeftEncoder.getPosition() + righEncoder.getPosition())/2;
    }

}
