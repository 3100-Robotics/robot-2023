package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmCommand extends CommandBase{
    
    // var setup
    Arm arm;
    double EncoderReading, speed;
    XboxController m_Controller;

    public ArmCommand(Arm arm, XboxController controller) {
        // typical stuff
        this.arm = arm;
        m_Controller = controller;
        addRequirements(this.arm);
    }

    @Override
    public void execute() {
        // un according to pid
        // EncoderReading = arm.GetEncoderRotation();
        // speed = arm.controller.calculate(EncoderReading);
        // arm.Run(speed);
        double speed = m_Controller.getRightX();
        if (speed < 0.1 && speed > -0.1) {
            speed = 0.03;
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