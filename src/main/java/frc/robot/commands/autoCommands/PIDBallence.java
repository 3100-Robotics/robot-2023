package frc.robot.commands.autoCommands;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class PIDBallence extends PIDCommand{
    //TODO make this work once the gyro library
    public PIDBallence(PIDController controller, DoubleSupplier measurementSource, double setpoint,
            DoubleConsumer useOutput, Subsystem[] requirements) {
        super(controller, measurementSource, setpoint, useOutput, requirements);
    }
    
}
