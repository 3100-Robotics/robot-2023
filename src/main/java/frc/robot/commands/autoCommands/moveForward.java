package frc.robot.commands.autoCommands;

import frc.robot.Constants.driveTrainConstants;
import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class moveForward extends CommandBase {
  private final Drive subsystem;
  private final double speed, distance;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public moveForward(Drive subsystem, double speed, double distance) {
    this.subsystem = subsystem;
    this.speed = speed;
    this.distance = distance * driveTrainConstants.feet2tick;
    
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subsystem.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   subsystem.arcadeDrive(speed, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subsystem.arcadeDrive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (speed < 0) {
      return subsystem.getAverageEncoderRotation() <= -distance;
    }
    else {
      return subsystem.getAverageEncoderRotation() >= distance;
    }
  }
}
