package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.endEffectorConstants;
import frc.robot.subsystems.Claw;

public class moveClaw extends CommandBase{


  Claw claw;
  double distance, speed;

  public moveClaw(Claw claw, double speed, double distance) {
    this.claw = claw;
    this.distance = distance * (endEffectorConstants.kRots2inches / 12);
    this.speed = speed;
    addRequirements(this.claw);
  }

  @Override
  public void execute() {
    claw.runLeft(speed);
    claw.runRight(speed);
  }

  @Override
  public boolean isFinished() {
    // am I finished?
    return claw.getLeftEncoder() >= distance;
  }
}
