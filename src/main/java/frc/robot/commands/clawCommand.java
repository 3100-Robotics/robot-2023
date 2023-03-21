package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class clawCommand extends CommandBase{
    
  // var setup
  Claw affector;
  XboxController controller;

  public clawCommand(XboxController controller, Claw affector) {
    // typical stuff
    this.controller = controller;
    this.affector = affector;
    addRequirements(this.affector);
  }

  public void execute(){
    // as long as joysticks aren't locked move them according to the input
    double leftSpeed = (controller.getYButton()) ? 0.5 : ((controller.getXButton()) ? -0.5 : 0);
    double rightSpeed = (controller.getBButton()) ? 0.5 : ((controller.getAButton()) ? -0.5 : 0);

    affector.runLeft(leftSpeed);
    affector.runRight(rightSpeed);
  }
}
