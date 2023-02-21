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

    @Override
    public void execute() {
      double xSpeed = m_controller.getLeftY();
      double zRotation = m_controller.getRightX();

      if (m_drive.slowmode) {
        xSpeed *= 0.5;
        zRotation *= 0.5;
      }

      m_drive.arcadeDrive(xSpeed, zRotation);
    }
}
