package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class moveArmFancy extends CommandBase{

    // var setup
    Arm arm;
    double distance, speed = 0;

    public moveArmFancy(Arm arm, double speed, double distance) {
        // typical stuff
        this.arm = arm;
        this.distance = distance;
        this.speed = speed;
        addRequirements(this.arm);
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("relative distance", distance);
        double currDistance = arm.GetEncoderRotation();
        SmartDashboard.putNumber("curr distance", currDistance);
        distance = currDistance - 2.5;
        SmartDashboard.putNumber("absolute distance", distance);
    }

    @Override
    public void execute() {
        arm.Run(speed);
    }

    @Override
    public void end(boolean interrupted) {
        arm.Stop();
    }

    @Override
    public boolean isFinished() {
        if (speed < 0) {
            return arm.GetEncoderRotation() <= distance;
        }
        else {
            return arm.GetEncoderRotation() >= distance;
        }
    }
}
