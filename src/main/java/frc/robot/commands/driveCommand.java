package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IOConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Drive;

public class driveCommand extends CommandBase{

  // var setup
  private final Drive m_drive;
  private final Elevator m_elevator;
  private final XboxController m_controller;

  double xSpeed, zRotation;

  public driveCommand(Drive drive, Elevator elevator, XboxController controller) {
    // typical stuff
    m_drive = drive;
    m_elevator = elevator;
    m_controller = controller;
    addRequirements(m_drive);
  }

  @Override
  public void execute() {
    // get speed
    // double xSpeed = m_controller.getRawAxis(IOConstants.leftYAxisChannel);
    xSpeed = -m_controller.getRawAxis(IOConstants.leftYAxisChannel);
    zRotation = -m_controller.getRawAxis(IOConstants.rightXAxisChannel);
    // double zRotation = m_controller.getRightX();

    // apply slow mode

    if (m_controller.getRightTriggerAxis() > 0.5) {
      xSpeed *= 1;
      zRotation *= 0.65;
    }
    else {
      xSpeed *= 0.5;
      zRotation *= 0.25;
    }

    xSpeed *= 1.4 - Math.abs(m_elevator.GetEncoderRotation()*0.01);


    // if (m_drive.slowmode) {
    // }
    // else {
    // }

    // if (xSpeed > 0) {
    //   xSpeed = Math.pow(xSpeed, 2);
    // }
    // else {
    //   xSpeed = -Math.pow(xSpeed, 2);
    // }
    // if (zRotation > 0) {
    //   zRotation = Math.pow(zRotation, 2);
    // }
    // else {
    //   zRotation = -Math.pow(zRotation, 2);
    // }

    // drive robot
    m_drive.arcadeDrive(xSpeed, zRotation);
  }

  
}
