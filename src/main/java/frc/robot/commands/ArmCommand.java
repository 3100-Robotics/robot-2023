package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmMotorConstants;
import frc.robot.subsystems.Arm;

public class ArmCommand extends CommandBase{
    Arm arm;
    PIDController controller1, controller2, controller3;
    double speed;
    double distance;

    void Initialize(){
        arm.resetEncoders();
    }

    public ArmCommand(double speed, double distance, Arm arm){
        controller1 = new PIDController(ArmMotorConstants.kp, ArmMotorConstants.ki, ArmMotorConstants.kd);
        controller1.setSetpoint(ArmMotorConstants.lvl1);
        this.speed = speed;
        this.distance = distance;
    }
    
    public void execute(){
        double sensorPosition = arm.GetEncoderRotation() * ArmMotorConstants.tick2Feet;
        double moveSpeed = controller1.calculate(sensorPosition, distance);

        arm.Run(moveSpeed);
    }

    public boolean isFinished(){
        return controller1.atSetpoint();
    }

}