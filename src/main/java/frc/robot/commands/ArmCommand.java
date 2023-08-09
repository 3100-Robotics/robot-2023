package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Arm;

public class ArmCommand extends CommandBase{
    
    // var setup
    Arm arm;
    CommandXboxController controller;

    public ArmCommand(Arm arm, CommandXboxController controller) {
        // typical stuff
        this.arm = arm;
        this.controller = controller;
        SmartDashboard.putNumber("arm/elevator speed", 0.5);
        addRequirements(this.arm);
    }

    @Override
    public void execute() {
        double speed = controller.getRightX();
        if (speed < 0.1 && speed > -0.1) {
            speed = 0.02;
        }
        else {
            speed *= SmartDashboard.getNumber("arm/elevator speed", 0.5);
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