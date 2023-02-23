package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase{
    private CANSparkMax LeftElevatorMotor = new CANSparkMax(ElevatorConstants.LeftElevatorMotor, MotorType.kBrushless);
    private CANSparkMax RightElevatorMotor = new CANSparkMax(ElevatorConstants.RightElevatorMotor, MotorType.kBrushless);

    private AbsoluteEncoder encoder = LeftElevatorMotor.getAbsoluteEncoder(Type.kDutyCycle);

    public PIDController controller;
    int setpointNum;
    double speed;

    public Elevator(){
        setpointNum = 1;
        controller = new PIDController(ElevatorConstants.kp, ElevatorConstants.ki, ElevatorConstants.kd);
        controller.setSetpoint(ElevatorConstants.elevatorLevels[setpointNum - 1]);
        LeftElevatorMotor.setIdleMode(IdleMode.kBrake);
        RightElevatorMotor.setIdleMode(IdleMode.kBrake);
        RightElevatorMotor.follow(LeftElevatorMotor, true);
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

    public void setSetpoint(double setpoint) {
        controller.setSetpoint(setpoint);
    }

    public double calculate(double measurement) {
        return controller.calculate(measurement);
    }

    public void Run(double speed) {
        LeftElevatorMotor.set(speed);
    }

    public void Stop(){
        LeftElevatorMotor.stopMotor();
    }

    public double GetEncoderRotation(){
        return encoder.getPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("motor 1 voltage", LeftElevatorMotor.getBusVoltage());
        SmartDashboard.putNumber("motor 2 voltage", LeftElevatorMotor.getBusVoltage());
        SmartDashboard.putNumber("elevator pos", encoder.getPosition());
        SmartDashboard.putNumber("elevator Speed", speed);
    }
}

