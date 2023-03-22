package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class moveTurn extends CommandBase{

  // var setup
  Drive Drive;
  double angle, speed, startAngle;

  public moveTurn(Drive Drive, double speed, double angle) {
    // typical stuff
    this.Drive = Drive;
    this.angle = angle;
    this.speed = speed;
    Drive.setSetpoint(Drive.getgyroz() + angle);
  }

  @Override
  public void initialize() {
    startAngle = Drive.getgyroz();
  }

  @Override
  public void execute() {
    // move
    Drive.arcadeDrive(0, speed);
  }

  @Override
  public boolean isFinished() {
    // am I done?
    if (speed < 0) {
      return (startAngle - Drive.getgyroy() <= angle);
    }
    else {
      return (startAngle - Drive.getgyroy() >= angle);
    }
  }
}
