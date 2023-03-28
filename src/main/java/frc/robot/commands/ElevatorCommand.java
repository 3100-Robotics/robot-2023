package frc.robot.commands;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class ElevatorCommand extends CommandBase{

    // var setup
    Elevator elevator;
    XboxController controller;

    public ElevatorCommand(Elevator elevator, XboxController controller) {
        // typical stuff
        this.elevator = elevator;
        this.controller = controller;
        addRequirements(this.elevator);
    }

    @Override
    public void execute() {
        double speed = -controller.getLeftY();
        if (speed < 0.1 && speed > -0.1) {
            speed = 0.03;
        }
        elevator.Run(speed);
    }

    @Override
    public boolean isFinished() {
        // am I finished?
        // return elevator.atSetpoint();
        return false;
    }
    
}
