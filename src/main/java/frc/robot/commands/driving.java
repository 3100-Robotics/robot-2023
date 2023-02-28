package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IOConstants;
import frc.robot.subsystems.drivetrain;

public class driving extends CommandBase{

  // var setup
  private final drivetrain m_drive;
  private final XboxController m_controller;

  public driving(drivetrain subsystem, XboxController controller) {
    // typical stuff
    m_drive = subsystem;
    m_controller = controller;
    addRequirements(m_drive);
  }

  @Override
  public void execute() {
    // get speed
    // double xSpeed = m_controller.getRawAxis(IOConstants.leftYAxisChannel);
    double xSpeed = m_controller.getRawAxis(IOConstants.leftYAxisChannel);
    double zRotation = m_controller.getRawAxis(IOConstants.rightXAxisChannel);
    // double zRotation = m_controller.getRightX();

    // apply slow mode
    if (m_drive.slowmode) {
      xSpeed *= 0.5;
      zRotation *= 0.5;
    }

    // drive robot
    m_drive.arcadeDrive(xSpeed, zRotation);
  }
}
