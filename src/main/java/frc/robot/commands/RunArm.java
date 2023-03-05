package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Arm;

public class RunArm extends CommandBase {
  private final Arm m_subsystem;

  private final double speed;
  private double setpoint;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RunArm(Arm subsystem, double speed) {
    m_subsystem = subsystem;
    this.speed = speed;
    setpoint = 0;
    
    addRequirements(subsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (m_subsystem.position) {
      case ("ground"):
        setpoint = ElevatorConstants.groundRots;
      case ("mid"):
        setpoint = ElevatorConstants.midConeRots;
      case ("player"):
        setpoint = ElevatorConstants.humanPlayerRots;
      case ("high"):
        setpoint = ElevatorConstants.highConeRots;
    }
    if (m_subsystem.GetEncoderRotation() >= setpoint) {
      m_subsystem.Run(0.02);
    }
    else {
        m_subsystem.Run(speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.Stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
