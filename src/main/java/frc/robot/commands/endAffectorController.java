package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.endAffector;

public class endAffectorController extends CommandBase{
    
    endAffector affector;
    XboxController controller;

    public endAffectorController(XboxController controller, endAffector affector) {
        this.controller = controller;
        this.affector = affector;
        addRequirements(this.affector);
    }

    private double limit(double value) {
        if (value >= +0.1)
          return value;
    
        if (value <= -0.1)
          return value;
        
        return 0;
      }

    public void execute(){
      if (!affector.endAffectorLock) {
        affector.runLeft(limit(controller.getLeftX()));
        affector.runRight(limit(controller.getRightX()));
      }
    }

}
