package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase{
    // Motors
    private final CANSparkMax leftMotor = new CANSparkMax(ElevatorConstants.LeftElevatorMotor, MotorType.kBrushless);
    private final CANSparkMax rightMotor = new CANSparkMax(ElevatorConstants.RightElevatorMotor, MotorType.kBrushless);

    // Encoder
    private final RelativeEncoder internalEncoder = leftMotor.getEncoder(); // Only use the encoder off of the left NEO

    // Limiter to ensure that the arm does not accelerate too rapidly
    private final SlewRateLimiter limiter = new SlewRateLimiter(ElevatorConstants.slewRate);

    // pid things
    public PIDController controller;
    int setpointNum;
    double speed;

    public Elevator(){
        // pid
        setpointNum = 1;
        controller = new PIDController(ElevatorConstants.kp, ElevatorConstants.ki, ElevatorConstants.kd);
//        controller.setSetpoint(ElevatorConstants.elevatorLevels[setpointNum - 1]);

        // Configure motors
        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.follow(leftMotor, true);

        leftMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);
        leftMotor.setSoftLimit(SoftLimitDirection.kForward, 84);
        leftMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        leftMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    }

    /**
     * Set where the elevator should be positioned
     * @param position How high the elevator should be (between 0 and 1, 0 being floor, 1 being max)
     */
    public void setTargetPosition(double position) {
        // Check that the position is valid
        if (position > 1 || position < 0) {
            System.out.println("Trying to set elevator to invalid position");
            return;
        }
    }

    public double getTargetPosition() {
        return 0.0;
    }

    public boolean atSetpoint() {
        // am I at the pid setpoint?
        return controller.atSetpoint();
    }

    public void run(double speed) {
        // set motor speed
        leftMotor.set(limiter.calculate(speed));
    }

    public void stop(){
        // stop elevator
        leftMotor.stopMotor();
    }

    public double GetEncoderRotation(){
        // get the pos of the elevator
        // return encoder.getPosition();
        return internalEncoder.getPosition();
    }

    @Override
    public void periodic() {
        // useful data
        Shuffleboard.selectTab("debug");
//        SmartDashboard.putNumber("elevator pos", GetEncoderRotation());
//
//        SmartDashboard.putNumber("elevator Speed", speed);
    }
}

