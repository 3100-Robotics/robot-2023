package frc.robot.commands;

import frc.robot.subsystems.mover;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class moveConMover extends CommandBase {
  private final mover m_subsystem;
  private final double[] m_distances;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public moveConMover(mover subsystem, double[] distances) {
    m_subsystem = subsystem;
    m_distances = distances;
    
    addRequirements(m_subsystem);
  }

  @Override
  public void initialize() {
    m_subsystem.setSetpoints(m_distances);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Pose2d currPos = m_subsystem.getPos();   
    // double[] calculations = {m_distances[0] - currPos.getX(), m_distances[1] = currPos.getY()};
    // double[] speeds = m_subsystem.calculate(calculations);
    // m_subsystem.move(speeds);
    double[] speed = {m_distances[2], m_distances[2]};
    m_subsystem.move(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    double[] speeds = {0, 0};
    m_subsystem.move(speeds);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean[] atSetpoint = m_subsystem.atSetpoint();
    if (atSetpoint[0] && atSetpoint[1]) {
        return true;
    }
    return false;
  }
}
