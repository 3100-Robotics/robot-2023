package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.endAffectorConstants;
import frc.robot.subsystems.endAffector;

public class openAffector extends CommandBase{


  endAffector affector;
  double distance, speed;

  public openAffector(endAffector affector, double speed, double distance) {
    this.affector = affector;
    this.distance = distance * (endAffectorConstants.kRots2inches / 12);
    this.speed = speed;
    addRequirements(this.affector);
  }

  @Override
  public void execute() {
    affector.runLeft(speed);
  }

  @Override
  public boolean isFinished() {
    // am I finished?
    if (affector.getLeftEncoder() >= distance) {
      return true;
    }
    return false;
  }
}
