package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmCommand extends CommandBase{
    
    // var setup
    Arm arm;
    double EncoderReading, speed;

    public ArmCommand(Arm arm) {
        // typical stuff
        this.arm = arm;
        addRequirements(this.arm);
    }

    @Override
    public void execute() {
        // un according to pid
        EncoderReading = arm.GetEncoderRotation();
        speed = arm.controller.calculate(EncoderReading);
        arm.Run(speed);
    }

    @Override
    public boolean isFinished() {
        // am I finished?
        return arm.controller.atSetpoint();
    }
}