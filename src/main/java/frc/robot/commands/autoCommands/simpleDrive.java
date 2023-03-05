package frc.robot.commands.autoCommands;

import frc.robot.Constants.driveTrainConstants;
import frc.robot.subsystems.drivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class simpleDrive extends CommandBase {
  private final drivetrain m_subsystem;
  private final double speed, distance;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public simpleDrive(drivetrain subsystem, double speed, double distance) {
    m_subsystem = subsystem;
    this.speed = speed;
    this.distance = distance * driveTrainConstants.feet2tick;
    
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   m_subsystem.arcadeDrive(speed, 0); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.arcadeDrive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putNumber("auton distance", m_subsystem.getAverageEncoderRotation()*driveTrainConstants.tick2feet);
    if (speed < 0) {
        if (m_subsystem.getAverageEncoderRotation() <= -distance) {
            return true;
        }
    }
    else {
        if (m_subsystem.getAverageEncoderRotation() >= distance) {
            return true;
        }
    }
    return false;
  }
}
