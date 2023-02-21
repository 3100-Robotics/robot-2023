package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class ElevatorCommand extends CommandBase{

    Elevator elevator;
    double EncoderReading;

    public ElevatorCommand(Elevator elevator) {
        this.elevator = elevator;
        addRequirements(this.elevator);
    }

    @Override
    public void execute() {
        EncoderReading = elevator.GetEncoderRotation();
        double speed = elevator.calculate(EncoderReading);
        elevator.Run(speed);
    }

    @Override
    public boolean isFinished() {
        return elevator.atSetpoint();
    }
    
}
