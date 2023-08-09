package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Claw;

public class clawCommand extends CommandBase{
    
  // var setup
  Claw claw;
  CommandXboxController controller;

  public clawCommand(CommandXboxController controller, Claw claw) {
    // typical stuff
    this.controller = controller;
    this.claw = claw;
    addRequirements(this.claw);
  }

  public void execute(){
    // as long as joysticks aren't locked move them according to the input
    double leftSpeed = (controller.y().getAsBoolean()) ? 0.5 : ((controller.x().getAsBoolean()) ? -0.5 : 0);
    double rightSpeed = (controller.b().getAsBoolean()) ? 0.5 : ((controller.a().getAsBoolean()) ? -0.5 : 0);

    claw.runLeft(leftSpeed);
    claw.runRight(rightSpeed);
  }
}
