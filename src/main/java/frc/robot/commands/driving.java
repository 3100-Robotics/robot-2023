package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain;

public class driving extends CommandBase{
    private final drivetrain m_drive;
    private final XboxController m_controller;

    public driving(drivetrain subsystem, XboxController controller) {
        m_drive = subsystem;
        m_controller = controller;
        addRequirements(m_drive);
    }

    private double limit(double value) {
        if (value >= +0.1)
          return value;
    
        if (value <= -0.1)
          return value;
        
        return 0;
      }

    @Override
    public void execute() {
        double xSpeed = -m_controller.getLeftY();
        double zRotation = m_controller.getRightX();

        m_drive.arcadeDrive(limit(xSpeed), limit(zRotation));
    }
}
