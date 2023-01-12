package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.elevatorConstants;

public class Elevator extends SubsystemBase{
    private CANSparkMax elevatorMotor1 = new CANSparkMax(elevatorConstants.elevator1port, MotorType.kBrushless);
    private CANSparkMax elevatorMotor2 = new CANSparkMax(elevatorConstants.elevator2port, MotorType.kBrushless);

    private RelativeEncoder encoder = elevatorMotor1.getEncoder();

    public Elevator() {
        elevatorMotor2.follow(elevatorMotor1);
        elevatorMotor1.setIdleMode(IdleMode.kBrake);
        elevatorMotor2.setIdleMode(IdleMode.kBrake);
    }

    public void run(double speed){
        elevatorMotor1.set(speed);
    }
    public void stop(){
        elevatorMotor1.stopMotor();
    }
    
    public double getEncoderRotation() {
        return encoder.getPosition();
    }
}
