package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class MoveElevator extends CommandBase{

    // var setup
    Elevator elevator;
    double distance, speed;

    public MoveElevator(Elevator elevator, double speed, double distance) {
        // typical stuff
        this.elevator = elevator;
        this.distance = distance;
        System.out.println(distance);
        this.speed = speed;
        addRequirements(this.elevator);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        elevator.Run(speed);
    }

    @Override
    public void end(boolean interupted) {
        elevator.Stop();
    }

    @Override
    public boolean isFinished() {
        if (speed < 0) {
            if (elevator.GetEncoderRotation() <= -distance) {
                return true;
            }
        }
        else {
            if (elevator.GetEncoderRotation() >= distance) {
                return true;
            }
        }
        return false;
    }
}
