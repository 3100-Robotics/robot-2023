package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class ElevatorCommand extends CommandBase{

    Elevator elevator;
    PIDController controller1, controller2, controller3;
    double EncoderReading, speed;
    XboxController gamController;

    public ElevatorCommand(Elevator elevator, XboxController gameController) {
        this.elevator = elevator;
        controller1 = new PIDController(ElevatorConstants.kp, ElevatorConstants.ki, ElevatorConstants.kd);
        controller1.setSetpoint(ElevatorConstants.lvl1);
    }

    @Override
    public void execute() {
        EncoderReading = elevator.GetEncoderRotation();
        speed = controller1.calculate(EncoderReading);
        elevator.Run(speed);
    }

    @Override
    public boolean isFinished() {
        return controller1.atSetpoint();
    }
    
}
