package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class MoveElevator extends CommandBase{

    // var setup
    Elevator elevator;
    boolean down;

    public MoveElevator(Elevator elevator, boolean isDown) {
        // typical stuff
        this.elevator = elevator;
        this.down = isDown;
    }

    @Override
    public void initialize() {
        // run the command
        elevator.incrementSetpoint(down);
    }

    @Override
    public boolean isFinished() {
        // finished after one run
        return true;
    }
}
