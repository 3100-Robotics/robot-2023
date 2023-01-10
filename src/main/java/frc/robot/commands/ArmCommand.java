package frc.robot.commands;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Arm;

public class ArmCommand extends PIDCommand{

    private Arm arm;

    public ArmCommand(PIDController controller, DoubleSupplier measurementSource, double setpoint,
            DoubleConsumer useOutput, Subsystem[] requirements) {
        super(controller, measurementSource, setpoint, useOutput, requirements);
        //TODO Auto-generated constructor stub
    }
    
    

}
