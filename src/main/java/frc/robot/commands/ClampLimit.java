package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.endAffector;

public class ClampLimit extends CommandBase{
    
    static endAffector ea;
    double clampSetting = 1;
    String std = "cube";
    public static boolean clamped = false;
    XboxController controller;

    public ClampLimit(endAffector ea, XboxController controller){
        this.ea = ea;
        this.controller = controller;
    }

    public void execute(){
        if(clampSetting == 1){
            std = "Cube";
            if(ea.getLeftEncoder() - ea.getRightEncoder() < 5){
                clamped = true;
            }
        }
        else if(clampSetting == 2){
            std = "Cone";
            if(ea.getLeftEncoder() - ea.getRightEncoder() < 3){
                clamped = true;
            }
        }
        else if(clampSetting == 3){
            std = "Tipped Cone";
            if(ea.getLeftEncoder() - ea.getRightEncoder() < 1){
                clamped = true;
            }
        }

        SmartDashboard.putString("What to grab", std);
        SmartDashboard.putBoolean("Clammped", clamped);

        if(clamped == false){
            double leftSpeed = controller.getLeftX();
            double rightSpeed = controller.getRightX();
            
            ea.runLeft(limit(leftSpeed));
            ea.runRight(limit(rightSpeed));
        }
    }

    public void incrementClampSetting(){
        clampSetting += 1;
        if(clampSetting == 4){
            clampSetting = 1;
        }
    }

    public static void open(){
        ea.runBothOpposite(-1);
        clamped = false;
    }

    private double limit(double value) {
        if (value >= +0.1)
          return value;
    
        if (value <= -0.1)
          return value;
        
        return 0;
      }

}
