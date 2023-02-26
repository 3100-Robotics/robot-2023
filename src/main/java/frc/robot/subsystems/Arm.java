package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmMotorConstants;

public class Arm extends SubsystemBase{
    // motor
    private CANSparkMax armMotor = new CANSparkMax(ArmMotorConstants.armMotor, MotorType.kBrushless);

    // thorugh bore encoder
    private AbsoluteEncoder encoder = armMotor.getAbsoluteEncoder(Type.kDutyCycle);

    // pid
    public PIDController controller;
    int setpointNum;

    public Arm() {
        // pid config
        setpointNum = 1;
        controller = new PIDController(ArmMotorConstants.kp, ArmMotorConstants.ki, ArmMotorConstants.kd);
        controller.setSetpoint(ArmMotorConstants.armLevels[setpointNum - 1]);

        // motor config
        armMotor.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("arm pos", encoder.getPosition());
    }

    public void incrementSetpoint(boolean negative) {
        // change setpoint to next pre-defined setpoint
        if (negative) {
            if (setpointNum != 1) {
                setpointNum -= 1;
            }
        }
        else {
            if (setpointNum != ArmMotorConstants.armLevels.length){
                setpointNum += 1;
            }
        }
        controller.setSetpoint(ArmMotorConstants.armLevels[setpointNum - 1]);
    }

    public boolean atSetpoint() {
        // am I at setpoint
        return controller.atSetpoint();
    }

    public void setSetpoint(double setpoint) {
        // set the setpoint manually
        controller.setSetpoint(setpoint);
    }

    public void Run(double speed) {
        // run the motor
        armMotor.set(speed);
    }

    public void Stop(){
        // stop the motor
        armMotor.stopMotor();
    }

    public double GetEncoderRotation(){
        // get the encoder rot
        return encoder.getPosition();
    }
}
