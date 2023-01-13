package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.endAffector;

public class endAffectorController extends CommandBase{
    
    endAffector ea;
    XboxController controller;

    public endAffectorController(XboxController controller) {
        this.controller = controller;
    }

    public void execute(){
        ea.runLeft(controller.getLeftX());
        ea.runRight(controller.getRightX());
    }

}
