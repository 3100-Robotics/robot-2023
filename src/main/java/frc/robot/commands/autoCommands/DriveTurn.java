package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain;

public class DriveTurn extends CommandBase{

  // var setup
  drivetrain Drive;
  double angle;

  public DriveTurn(drivetrain Drive, double angle) {
    // typical stuff
    this.Drive = Drive;
    this.angle = angle;
    Drive.setSetpoint(Drive.getgyroz() + angle);
  }

  @Override
  public void execute() {
    // move
    Drive.arcadeDrive(Drive.driveCalculate(Drive.getAverageEncoderRotation()), 0);
  }

  @Override
  public boolean isFinished() {
    // am I done?
    return Drive.atSetpoint();
  }
}
