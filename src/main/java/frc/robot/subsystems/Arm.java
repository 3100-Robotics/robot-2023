package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmMotorConstants;

public class Arm extends SubsystemBase{
    private CANSparkMax armMotor = new CANSparkMax(ArmMotorConstants.armMotor1, MotorType.kBrushless);

    private AbsoluteEncoder encoder = armMotor.getAbsoluteEncoder(Type.kDutyCycle);

    public PIDController controller;
    int setpointNum;

    public Arm() {
        controller = new PIDController(ArmMotorConstants.kp, ArmMotorConstants.ki, ArmMotorConstants.kd);
        setpointNum = 1;
    }

    public void incrementSetpoint(boolean negative) {
        if (negative) {
            if (setpointNum != 1) {
                setpointNum -= 1;
            }
        }
        else {
            if (setpointNum != ArmMotorConstants.numLevels){
                setpointNum += 1;
            }
        }
        if (setpointNum == 1) {
            controller.setSetpoint(ArmMotorConstants.shelflvl1);
        }
        else if (setpointNum == 2) {
            controller.setSetpoint(ArmMotorConstants.shelflvl2);
        }
        else {
            controller.setSetpoint(ArmMotorConstants.shelflvl3);
        }
    }

    public boolean atSetpoint() {
        return controller.atSetpoint();
    }

    public void Run(double speed) {
        armMotor.set(speed);
    }

    public void Stop(){
        armMotor.stopMotor();
    }

    public double GetEncoderRotation(){
        return encoder.getPosition();
    }
}
