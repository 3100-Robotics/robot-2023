package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class MoveElevator extends CommandBase{

    // var setup
    Elevator elevator;
    double distance, speed;

    public MoveElevator(Elevator elevator, double speed, double distance) {
        // typical stuff
        this.elevator = elevator;
        this.distance = distance * (ElevatorConstants.kInches2Rots);
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
        // finished after one run
        if (elevator.GetEncoderRotation() >= distance) {
            return true;
        }
        return false;
    }
}
