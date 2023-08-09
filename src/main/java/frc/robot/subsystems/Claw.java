package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.endEffectorConstants;

public class Claw extends SubsystemBase{
    // motors
    private final CANSparkMax leftEffector = new CANSparkMax(endEffectorConstants.leftMotor, MotorType.kBrushless);
    private final CANSparkMax rightEffector = new CANSparkMax(endEffectorConstants.rightMotor, MotorType.kBrushless);

    private final RelativeEncoder LefEncoder = leftEffector.getEncoder();
    private final RelativeEncoder RightEncoder = rightEffector.getEncoder();

    // are the joystick controls locked?
    public Boolean endEffectorLock = false;

    public Claw() {
        leftEffector.setInverted(false);
        rightEffector.setInverted(false);

        leftEffector.setIdleMode(IdleMode.kBrake);
        rightEffector.setIdleMode(IdleMode.kBrake);

        leftEffector.setSoftLimit(SoftLimitDirection.kForward, endEffectorConstants.leftSoftLimitRots);
        leftEffector.enableSoftLimit(SoftLimitDirection.kForward, true);

        rightEffector.setSoftLimit(SoftLimitDirection.kForward, endEffectorConstants.rightSoftLimitRots);
        rightEffector.enableSoftLimit(SoftLimitDirection.kForward, true);
    }

    @Override
    public void periodic() {
        // useful details
        Shuffleboard.selectTab("debug");
//        SmartDashboard.putNumber("left encoder", getLeftEncoder());
//        SmartDashboard.putNumber("right encoder", getRightEncoder());
    }

    public void toggleEndEffectorLock() {
        // lock/unlock the joystick controls
        endEffectorLock = !endEffectorLock;
    }

    /**
    * @param speed speed to set the left claw to
    */  
    public void runLeft(double speed) {
        // run the left claw
        leftEffector.set(speed);
    }

    /**
    * @param speed speed to set the right claw to
    */  
    public void runRight(double speed) {
        // run the right claw
        rightEffector.set(speed);
    }

    public void stopLeft() {
        // stop the left claw
        leftEffector.stopMotor();
    }

    public void stopRight() {
        // stop the right claw
        rightEffector.stopMotor();
    }

    /**
    * @param speed the speed to move both claws to the left
    */  
    public void runBoth(double speed, double speedDiff) {
        // run both claws positive number = to the left
        leftEffector.set(speed);
        rightEffector.set(-speed-speedDiff);
    }

    public void stopBoth() {
        // stop both claws
        leftEffector.stopMotor();
        rightEffector.stopMotor();
    }

    public double getLeftEncoder() {
        // get the left encoder pos
        return LefEncoder.getPosition();
    }

    public double getRightEncoder() {
        // get the right encoder pos
        return RightEncoder.getPosition();
    }

    public double getCenterPos() {
        // get the center point between the claws
        return (LefEncoder.getPosition() + RightEncoder.getPosition())/2;
    }

    public Command runLeftCommand(double speed) {
        return this.run(() -> runLeft(speed));
    }

    public Command runRightCommand(double speed) {
        return this.run(() -> runRight(speed));
    }

    public Command runBothOut(double speed, double speedDiff) {
        return this.run(() -> runBoth(speed, speedDiff));
    }
}
