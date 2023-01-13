package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class ElevatorCommand extends CommandBase{

    Elevator elevator;
    PIDController controller;
    double EncoderReading, speed, setpoint;
    int contollernum;

    public ElevatorCommand(Elevator elevator) {
        this.elevator = elevator;
        contollernum = 1;
        controller = new PIDController(ElevatorConstants.kp, ElevatorConstants.ki, ElevatorConstants.kd);
    }

    public void incrementcontroller(boolean negative) {
        if (negative) {
            if (contollernum != 1) {
                contollernum -= 1;
            }
        }
        else {
            if (contollernum != 3){
                contollernum += 1;
            }
        }
    }

    public boolean atSetpoint() {
        return controller.atSetpoint();
    }

    @Override
    public void execute() {
        EncoderReading = elevator.GetEncoderRotation();
        if (contollernum == 1) {
            setpoint = ElevatorConstants.lvl1;
        }
        else if (contollernum == 2) {
            setpoint = ElevatorConstants.lvl2;
        }
        else {
            setpoint = ElevatorConstants.lvl3;
        }
        speed = controller.calculate(EncoderReading, setpoint);
        elevator.Run(speed);
    }

    @Override
    public boolean isFinished() {
        return controller.atSetpoint();
    }
    
}
