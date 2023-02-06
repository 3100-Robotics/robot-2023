package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class ElevatorCommand extends CommandBase{

    Elevator elevator;
    double EncoderReading, speed;

    public ElevatorCommand(Elevator elevator) {
        this.elevator = elevator;
    }

    @Override
    public void execute() {
        EncoderReading = elevator.GetEncoderRotation();
        speed = elevator.controller.calculate(EncoderReading);
        elevator.Run(speed);
    }

    @Override
    public boolean isFinished() {
        return elevator.controller.atSetpoint();
    }
    
}
