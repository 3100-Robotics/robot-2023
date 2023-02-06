package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class MoveElevator extends CommandBase{
    Elevator elevator;
    boolean down;

    public MoveElevator(Elevator elevator, boolean isDown) {
        this.elevator = elevator;
        this.down = isDown;
    }

    @Override
    public void initialize() {
        elevator.incrementSetpoint(down);
    }

    @Override
    public boolean isFinished() {
        return elevator.atSetpoint();
    }
}
