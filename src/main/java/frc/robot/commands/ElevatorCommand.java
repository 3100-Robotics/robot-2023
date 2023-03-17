package frc.robot.commands;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class ElevatorCommand extends CommandBase{

    // var setup
    Elevator elevator;
    double EncoderReading;
    XboxController m_Controller;

    public ElevatorCommand(Elevator elevator, XboxController controller) {
        // typical stuff
        this.elevator = elevator;
        m_Controller = controller;
        addRequirements(this.elevator);
    }

    @Override
    public void execute() {
        // run according to pid
        // EncoderReading = elevator.GetEncoderRotation();
        // double speed = elevator.calculate(EncoderReading - elevator.getSetpoint());
        // SmartDashboard.putNumber("elevator speed", speed);
        // elevator.Run(speed);
        double speed = -m_Controller.getLeftY();
        if (speed < 0.1 && speed > -0.1) {
            speed = 0.03;
        }
        elevator.Run(speed*0.75);
    }

    @Override
    public boolean isFinished() {
        // am I finished?
        // return elevator.atSetpoint();
        return false;
    }
    
}
