package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmMotorConstants;

public class Arm extends SubsystemBase{
    private CANSparkMax am1 = new CANSparkMax(ArmMotorConstants.armMotor1, MotorType.kBrushless);

    private AbsoluteEncoder encoder = am1.getAbsoluteEncoder(Type.kDutyCycle);

    public Arm() {
        
    }

    void Initialize(){}

    public void Run(double speed) {
        am1.set(speed);
    }

    public void Stop(){
        am1.stopMotor();
    }

    public double GetEncoderRotation(){
        return encoder.getPosition();
    }
}
