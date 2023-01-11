package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase{
    private CANSparkMax em1 = new CANSparkMax(ElevatorConstants.elevatorMotor1, MotorType.kBrushless);
    private CANSparkMax em2 = new CANSparkMax(ElevatorConstants.elevatorMotor2, MotorType.kBrushless);

    private RelativeEncoder encoder = em1.getEncoder();

    public Elevator(){
        em2.follow(em1);
    }

    public void Run(double speed) {
        em1.set(speed);
    }

    public void Stop(){
        em1.stopMotor();
    }

    public double GetEncoderRotation(){
        return encoder.getPosition();
    }
}

