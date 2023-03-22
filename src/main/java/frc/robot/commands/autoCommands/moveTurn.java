package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class moveTurn extends CommandBase{

  // var setup
  Drive Drive;
  double angle;

  public moveTurn(Drive Drive, double angle) {
    // typical stuff
    this.Drive = Drive;
    this.angle = angle;
    Drive.setSetpoint(Drive.getGyroZ() + angle);
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
