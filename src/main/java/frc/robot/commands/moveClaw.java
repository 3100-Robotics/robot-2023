package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.endAffector;

public class moveClaw extends CommandBase{
    
  // var setup
  endAffector affector;
  XboxController controller;

  public moveClaw(XboxController controller, endAffector affector) {
    // typical stuff
    this.controller = controller;
    this.affector = affector;
    addRequirements(this.affector);
  }

  private double limit(double value) {
    // deadband
    if (value >= 0.1){
      return value;
    }

    if (value <= -0.1) {
      return value;
    }
    
    return 0;
  }

  public void execute(){
    // as long as joysticks aren't locked move them according to the input
    if (!affector.endAffectorLock) {
      affector.runLeft(-limit(controller.getLeftX()));
      affector.runRight(limit(controller.getRightX()));
    }
  }
}