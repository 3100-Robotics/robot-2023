package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase{
    private CANSparkMax elevatorMotor1 = new CANSparkMax(ElevatorConstants.elevatorMotor1, MotorType.kBrushless);
    private CANSparkMax elevatorMotor2 = new CANSparkMax(ElevatorConstants.elevatorMotor2, MotorType.kBrushless);

    private AbsoluteEncoder encoder = elevatorMotor1.getAbsoluteEncoder(Type.kDutyCycle);

    public PIDController controller;
    int setpointNum;

    public Elevator(){
        setpointNum = 1;
        controller = new PIDController(ElevatorConstants.kp, ElevatorConstants.ki, ElevatorConstants.kd);
        controller.setSetpoint(ElevatorConstants.elevatorLevels[setpointNum - 1]);
        elevatorMotor2.follow(elevatorMotor1);
    }

    public void incrementSetpoint(boolean negative) {
        if (negative) {
            if (setpointNum != 1) {
                setpointNum -= 1;
            }
        }
        else {
            if (setpointNum != ElevatorConstants.elevatorLevels.length){
                setpointNum += 1;
            }
        }

        controller.setSetpoint(ElevatorConstants.elevatorLevels[setpointNum - 1]);
    }

    public boolean atSetpoint() {
        return controller.atSetpoint();
    }

    public void Run(double speed) {
        elevatorMotor1.set(speed);
    }

    public void Stop(){
        elevatorMotor1.stopMotor();
    }

    public double GetEncoderRotation(){
        return encoder.getPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("motor 1 voltage", elevatorMotor1.getBusVoltage());
        SmartDashboard.putNumber("motor 2 voltage", elevatorMotor1.getBusVoltage());
        SmartDashboard.putNumber("motor speeds", encoder.getVelocity());
    }
}

