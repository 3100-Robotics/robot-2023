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
      double xSpeed = limit(m_controller.getLeftY());
      double zRotation = limit(m_controller.getRightX());

      if (xSpeed > 0) {
        xSpeed = Math.pow(xSpeed, 2);
      }
      else {
        xSpeed = Math.pow(xSpeed, 2) * -1;
      }

      if (zRotation > 0) {
        zRotation = Math.pow(zRotation, 2);
      }
      else {
        zRotation = Math.pow(zRotation, 2) * -1;
      }

      if (m_drive.slowmode) {
        xSpeed *= 0.5;
        zRotation *= 0.5;
      }

      m_drive.arcadeDrive(xSpeed, zRotation);
    }
}
