package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase{
    private static final double ENCODER_TICKS_TO_METERS = 1; // FIXME: Calculate using hardware. Use REV Hardware Client to measure ticks at top and divide by the max height of the elevator

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

    private double targetHeight;

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
     * @param height How high the elevator should be in meters
     */
    public void setTargetHeight(double height) {
        // TODO: Add check to make sure that it isn't trying to go too high or too low
        targetHeight = height;
    }

    public double getHeightMeters() {
        if (RobotBase.isSimulation()) { return targetHeight; } // Just feed back what it was told to do for simulation mode

        return internalEncoder.getPosition() * ENCODER_TICKS_TO_METERS;
    }

    public boolean atSetpoint() {
        // am I at the pid setpoint?
        return controller.atSetpoint();
    }

    @Deprecated
    public void run(double speed) {
        // set motor speed
        leftMotor.set(limiter.calculate(speed));
    }

    public void stop(){
        // stop elevator
        leftMotor.stopMotor();
    }

    @Deprecated
    public double GetEncoderRotation(){
        // get the pos of the elevator
        // return encoder.getPosition();
        return internalEncoder.getPosition();
    }



    @Override
    public void periodic() {
        // Set output based on target and current height
        leftMotor.set(
            controller.calculate(
                getHeightMeters(), 
                targetHeight)
        );

        // useful data
        Shuffleboard.selectTab("debug");
//        SmartDashboard.putNumber("elevator pos", GetEncoderRotation());
//
//        SmartDashboard.putNumber("elevator Speed", speed);
    }
}

