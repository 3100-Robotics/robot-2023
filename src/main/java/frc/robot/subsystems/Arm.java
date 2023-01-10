package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmMotorConstants;

public class Arm extends SubsystemBase{
    private CANSparkMax am1 = new CANSparkMax(ArmMotorConstants.armMotor1, MotorType.kBrushless);

    private RelativeEncoder encoder = am1.getEncoder();

    public Arm() {
        
    }

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
