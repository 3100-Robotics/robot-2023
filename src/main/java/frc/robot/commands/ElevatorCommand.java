package frc.robot.commands;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class ElevatorCommand extends PIDCommand{

    Elevator elevator;

    public ElevatorCommand(Double speed, double distance,
            Elevator elevator) {
                super(new PIDController(ElevatorConstants.kp, ElevatorConstants.ki, ElevatorConstants.kd), 
                elevator::GetEncoderRotation, distance, ouput -> elevator.Run(speed), elevator);
                this.elevator = elevator;
                getController().enableContinuousInput(-180, 180);
        
    }
    
}
