package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.endAffectorConstants;

public class Claw extends SubsystemBase{
    // motors
    private CANSparkMax leftAffector = new CANSparkMax(endAffectorConstants.leftMotor, MotorType.kBrushless);
    private CANSparkMax rightAffector = new CANSparkMax(endAffectorConstants.rightMotor, MotorType.kBrushless);

    // encoders for motors change to absolute when they exist
    // private AbsoluteEncoder LeftEncoder = leftAffector.getAbsoluteEncoder(Type.kDutyCycle);
    // private AbsoluteEncoder righEncoder = rightAffector.getAbsoluteEncoder(Type.kDutyCycle);
    private RelativeEncoder LefEncoder = leftAffector.getEncoder();
    private RelativeEncoder RighEncoder = rightAffector.getEncoder();

    // are the joystick controlls locked?
    public Boolean endAffectorLock = false;

    public Claw() {
        leftAffector.setInverted(false);
        rightAffector.setInverted(false);

        leftAffector.setIdleMode(IdleMode.kBrake);
        rightAffector.setIdleMode(IdleMode.kBrake);

        leftAffector.setSoftLimit(SoftLimitDirection.kForward, endAffectorConstants.leftSoftLimitRots);        
        leftAffector.enableSoftLimit(SoftLimitDirection.kForward, true);

        rightAffector.setSoftLimit(SoftLimitDirection.kForward, endAffectorConstants.rightSoftLimitRots);
        rightAffector.enableSoftLimit(SoftLimitDirection.kForward, true);
    }

    @Override
    public void periodic() {
        // useful details
        SmartDashboard.putNumber("left encoder", getLeftEncoder());
        SmartDashboard.putNumber("right encoder", getRightEncoder());
    }

    public void toggleEndAffectorLock() {
        // lock/unlock the joystick controlls
        endAffectorLock = !endAffectorLock;
    }

    /**
    * Constructor initializing hours to timeHours and 
    * minutes to timeMins. 
    * @param speed speed to set the left claw to
    */  
    public void runLeft(double speed) {
        // run the left claw
        leftAffector.set(speed);
    }

    /**
    * Constructor initializing hours to timeHours and 
    * minutes to timeMins. 
    * @param speed speed to set the right claw to
    */  
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

    /**
    * Constructor initializing hours to timeHours and 
    * minutes to timeMins. 
    * @param speed the speed to move both claws to the left
    */  
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
        return LefEncoder.getPosition();
    }

    public double getRightEncoder() {
        // get the right encoder pos
        return RighEncoder.getPosition();
    }

    public double getCenterPos() {
        // get the center point between the claws
        return (LefEncoder.getPosition() + RighEncoder.getPosition())/2;
    }
}
