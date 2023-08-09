package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.IOConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Drive;

public class driveCommand extends CommandBase{

  // var setup
  private final Drive drive;
  private final Elevator elevator;
  private final CommandXboxController controller;

  double xSpeed, zRotation;

  public driveCommand(Drive drive, Elevator elevator, CommandXboxController controller) {
    // typical stuff
    this.drive = drive;
    this.elevator = elevator;
    this.controller = controller;
    SmartDashboard.putNumber("drive speed", 0.5);
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

    if (SmartDashboard.getBoolean("fast mode", false)) {
      xSpeed *= 1;
      zRotation *= 0.55;
    }
    else {
      xSpeed *= SmartDashboard.getNumber("drive speed", 0.25);
//      xSpeed *= 0.5;
      zRotation *= 0.27;
    }

    xSpeed *= 1.4 - Math.abs(elevator.GetEncoderRotation()*0.01);

    // drive robot
    drive.arcadeDrive(xSpeed, zRotation);
  }

  
}
