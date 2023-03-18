package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase{
    // motors
    private CANSparkMax LeftElevatorMotor = new CANSparkMax(ElevatorConstants.LeftElevatorMotor, MotorType.kBrushless);
    private CANSparkMax RightElevatorMotor = new CANSparkMax(ElevatorConstants.RightElevatorMotor, MotorType.kBrushless);

    // encoder
    private RelativeEncoder internalEncoder = LeftElevatorMotor.getEncoder();

    private SlewRateLimiter limiter = new SlewRateLimiter(ElevatorConstants.slewRate);

    // pid things
    public PIDController controller;
    int setpointNum;
    double speed;

    public Elevator(){
        // pid
        setpointNum = 1;
        controller = new PIDController(ElevatorConstants.kp, ElevatorConstants.ki, ElevatorConstants.kd);
        controller.setSetpoint(ElevatorConstants.elevatorLevels[setpointNum - 1]);
        // motor config
        LeftElevatorMotor.setIdleMode(IdleMode.kBrake);
        RightElevatorMotor.setIdleMode(IdleMode.kBrake);
        // RightElevatorMotor.follow(LeftElevatorMotor, true);
    }

    public void incrementSetpoint(boolean negative) {
        // change the setpoint to the next one in the list
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
        // am I at the pid setpoint?
        return controller.atSetpoint();
    }

    public void setSetpoint(double setpoint) {
        // change that setpoint
        controller.setSetpoint(setpoint);
    }

    public double calculate(double measurement) {
        // use the pid to get speed
        return controller.calculate(measurement);
    }

    public double getSetpoint() {
        return controller.getSetpoint();
    }

    public void Run(double speed) {
        // set motor speed
        LeftElevatorMotor.set(limiter.calculate(speed));
    }

    public void Stop(){
        // stop elevator
        LeftElevatorMotor.stopMotor();
    }

    public double GetEncoderRotation(){
        // get the pos of the elevator
        // return encoder.getPosition();
        return internalEncoder.getPosition();
    }

    @Override
    public void periodic() {
        // useful data
        // SmartDashboard.putNumber("motor 1 voltage", LeftElevatorMotor.getBusVoltage());
        // SmartDashboard.putNumber("motor 2 voltage", LeftElevatorMotor.getBusVoltage());
        SmartDashboard.putNumber("elevator pos", GetEncoderRotation());
        
        SmartDashboard.putNumber("elevator Speed", speed);
    }
}

