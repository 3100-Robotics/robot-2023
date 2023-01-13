package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmMotorConstants;
import frc.robot.subsystems.Arm;

public class ArmCommand extends CommandBase{
    
    Arm arm;
    PIDController controller;
    double EncoderReading, speed, setpoint;
    int contollernum;

    public ArmCommand(Arm arm) {
        this.arm = arm;
        contollernum = 1;
        controller = new PIDController(ArmMotorConstants.kp, ArmMotorConstants.ki, ArmMotorConstants.kd);
    }

    public void incrementcontroller(boolean negative) {
        if (negative) {
            if (contollernum != 1) {
                contollernum -= 1;
            }
        }
        else {
            if (contollernum != 3){
                contollernum += 1;
            }
        }
    }

    @Override
    public void execute() {
        EncoderReading = arm.GetEncoderRotation();
        if (contollernum == 1) {
            setpoint = ArmMotorConstants.lvl1;
        }
        else if (contollernum == 2) {
            setpoint = ArmMotorConstants.lvl2;
        }
        else {
            setpoint = ArmMotorConstants.lvl3;
        }
        speed = controller.calculate(EncoderReading, setpoint);
        arm.Run(speed);
    }

    @Override
    public boolean isFinished() {
        return controller.atSetpoint();
    }
    

}