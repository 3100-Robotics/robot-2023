package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.ElevatorCommand;

public class MoveElevator extends CommandBase{
    ElevatorCommand elevatorcommand;
    boolean down;

    public MoveElevator(ElevatorCommand elevatorcommand, boolean isDown) {
        this.elevatorcommand = elevatorcommand;
        this.down = isDown;
    }

    @Override
    public void initialize() {
        elevatorcommand.incrementcontroller(down);
    }

    @Override
    public boolean isFinished() {
        return elevatorcommand.atSetpoint();
    }
}
