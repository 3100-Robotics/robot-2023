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

    private double limit(double value) {
        if (value >= +0.1)
          return value;
    
        if (value <= -0.1)
          return value;
        
        return 0;
      }

    public void execute(){
        ea.runLeft(limit(controller.getLeftX()));
        ea.runRight(limit(controller.getRightX()));
    }

}
