package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmCommand extends CommandBase{
    
    Arm arm;
    double EncoderReading, speed;

    public ArmCommand(Arm arm) {
        this.arm = arm;
        addRequirements(this.arm);
    }

    @Override
    public void execute() {
        EncoderReading = arm.GetEncoderRotation();
        speed = arm.controller.calculate(EncoderReading);
        arm.Run(speed);
    }

    @Override
    public boolean isFinished() {
        return arm.controller.atSetpoint();
    }
}