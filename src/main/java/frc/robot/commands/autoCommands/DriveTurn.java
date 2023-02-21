package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain;

public class DriveTurn extends CommandBase{
  drivetrain Drive;
  double angle;

  public DriveTurn(drivetrain Drive, double angle) {
      this.Drive = Drive;
      this.angle = angle;
  }

  @Override
  public void initialize() {
      Drive.setSetpoint(Drive.getgyroz() + angle);
  }

  @Override
  public void execute() {
      Drive.arcadeDrive(Drive.driveCalculate(Drive.getAverageEncoderRotation()), 0);
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return Drive.atSetpoint();
  }
}
