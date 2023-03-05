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

    public String position = "ground";
    public boolean altPos = false;

    public Elevator(){
        // pid
        setpointNum = 1;
        controller = new PIDController(ElevatorConstants.kp, ElevatorConstants.ki, ElevatorConstants.kd);
        controller.setSetpoint(0);
        // motor config
        LeftElevatorMotor.setIdleMode(IdleMode.kBrake);
        RightElevatorMotor.setIdleMode(IdleMode.kBrake);
        RightElevatorMotor.follow(LeftElevatorMotor, true);
    }

    // PID

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

    // movement

    public void Run(double speed) {
        // set motor speed
        LeftElevatorMotor.set(limiter.calculate(speed));
    }

    public void Stop(){
        // stop elevator
        LeftElevatorMotor.stopMotor();
    }

    public void setPos(String pos) {
        position = pos;
        altPos = false;
    }

    // data

    public double GetEncoderRotation(){
        // get the pos of the elevator
        return internalEncoder.getPosition();
    }

    @Override
    public void periodic() {
        // useful data
        // SmartDashboard.putNumber("motor 1 voltage", LeftElevatorMotor.getBusVoltage());
        // SmartDashboard.putNumber("motor 2 voltage", LeftElevatorMotor.getBusVoltage());
        SmartDashboard.putNumber("elevator pos", GetEncoderRotation());
    }
}

