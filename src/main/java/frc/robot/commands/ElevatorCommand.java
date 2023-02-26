package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class ElevatorCommand extends CommandBase{

    // var setup
    Elevator elevator;
    double EncoderReading;

    public ElevatorCommand(Elevator elevator) {
        // typical stuff
        this.elevator = elevator;
        addRequirements(this.elevator);
    }

    @Override
    public void execute() {
        // run according to pid
        EncoderReading = elevator.GetEncoderRotation();
        double speed = elevator.calculate(EncoderReading - elevator.getSetpoint());
        SmartDashboard.putNumber("elevator speed", speed);
        elevator.Run(speed);
    }

    @Override
    public boolean isFinished() {
        // am I finished?
        // return elevator.atSetpoint();
        return false;
    }
    
}
