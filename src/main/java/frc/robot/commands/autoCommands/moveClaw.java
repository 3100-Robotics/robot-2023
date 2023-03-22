package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.endAffectorConstants;
import frc.robot.subsystems.Claw;

public class moveClaw extends CommandBase{


  Claw affector;
  double distance, speed;

  public moveClaw(Claw affector, double speed, double distance) {
    this.affector = affector;
    this.distance = distance * (endAffectorConstants.kRots2inches / 12);
    this.speed = speed;
    addRequirements(this.affector);
  }

  @Override
  public void execute() {
    affector.runLeft(speed);
    affector.runRight(speed);
  }

  @Override
  public boolean isFinished() {
    // am I finished?
    if (speed < 0) {
      if (affector.getLeftEncoder() <= distance) {
        return true;
      }
    }
    else {
      if (affector.getLeftEncoder() >= distance) {
        return true;
      }
    }
    return false;
  }
}
