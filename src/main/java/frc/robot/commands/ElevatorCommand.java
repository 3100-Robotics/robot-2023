package frc.robot.commands;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Elevator;

public class ElevatorCommand extends CommandBase{

    // var setup
    Elevator elevator;
    CommandXboxController controller;

    public ElevatorCommand(Elevator elevator, CommandXboxController controller) {
        // typical stuff
        this.elevator = elevator;
        this.controller = controller;
        SmartDashboard.putNumber("arm/elevator speed", 0.5);
        addRequirements(this.elevator);
    }

    @Override
    public void execute() {
        double speed = -controller.getLeftY();
        if (speed < 0.1 && speed > -0.1) {
            speed = 0.03;
        }
        else {
            speed *= SmartDashboard.getNumber("arm/elevator speed", 0.5);
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
