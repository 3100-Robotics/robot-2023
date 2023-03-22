package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class MoveElevator extends CommandBase{

    // var setup
    Elevator arm;
    double distance, speed;

    public MoveElevator(Elevator arm, double speed, double distance) {
        // typical stuff
        this.arm = arm;
        this.distance = distance;
        this.speed = speed;
        addRequirements(this.arm);
    }

    @Override
    public void initialize() {}

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
