package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IOConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Drive;

public class driveCommand extends CommandBase{

  // var setup
  private final Drive drive;
  private final Elevator elevator;
  private final XboxController controller;

  double xSpeed, zRotation;

  public driveCommand(Drive drive, Elevator elevator, XboxController controller) {
    // typical stuff
    this.drive = drive;
    this.elevator = elevator;
    this.controller = controller;
    addRequirements(this.drive);
  }

  @Override
  public void execute() {
    // get speed
    // double xSpeed = controller.getRawAxis(IOConstants.leftYAxisChannel);
    xSpeed = -controller.getRawAxis(IOConstants.leftYAxisChannel);
    zRotation = -controller.getRawAxis(IOConstants.rightXAxisChannel);
    // double zRotation = controller.getRightX();

    // apply slow mode

    if (controller.getRightTriggerAxis() > 0.5) {
      xSpeed *= 1;
      zRotation *= 0.55;
    }
    else {
      xSpeed *= 0.5;
      zRotation *= 0.27;
    }

    xSpeed *= 1.4 - Math.abs(elevator.GetEncoderRotation()*0.01);

    // drive robot
    drive.arcadeDrive(xSpeed, zRotation);
  }

  
}
