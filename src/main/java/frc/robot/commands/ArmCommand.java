package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmCommand extends CommandBase{
    
    // var setup
    Arm arm;
    XboxController controller;

    public ArmCommand(Arm arm, XboxController controller) {
        // typical stuff
        this.arm = arm;
        this.controller = controller;
        addRequirements(this.arm);
    }

    @Override
    public void execute() {
        double speed = controller.getRightX();
        if (speed < 0.1 && speed > -0.1) {
            speed = 0.02;
        }
        arm.Run(speed);
    }

    @Override
    public boolean isFinished() {
        // am I finished?
        // return arm.controller.atSetpoint();
        return false;
    }
}