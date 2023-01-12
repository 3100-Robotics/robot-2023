package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class ElevatorCommand extends CommandBase{

    Elevator elevator;
    PIDController controller1, controller2, controller3;

    public ElevatorCommand(Double speed, double distance, Elevator elevator) {
        
        controller1 = new PIDController(ElevatorConstants.kp, ElevatorConstants.ki, ElevatorConstants.kd);
        controller1.setSetpoint(ElevatorConstants.lvl1);
    }
    
}
