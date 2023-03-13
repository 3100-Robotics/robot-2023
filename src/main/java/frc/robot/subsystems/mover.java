package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.moveConMover;

public class mover extends SubsystemBase {

  // elevator
  private CANSparkMax LeftElevatorMotor = new CANSparkMax(ElevatorConstants.LeftElevatorMotor, MotorType.kBrushless);
  private CANSparkMax RightElevatorMotor = new CANSparkMax(ElevatorConstants.RightElevatorMotor, MotorType.kBrushless);
  
  private CANSparkMax armMotor = new CANSparkMax(ArmConstants.armMotor, MotorType.kBrushless);

  private RelativeEncoder elevatorEncoder = LeftElevatorMotor.getEncoder();
  private RelativeEncoder armEncoder = armMotor.getEncoder();

  // private SlewRateLimiter limiter = new SlewRateLimiter(ElevatorConstants.slewRate);
  // private SlewRateLimiter limiter = new SlewRateLimiter(ArmMotorConstants.slewRate);

  public PIDController elevatorController;
  public PIDController armController;

  /** Creates a new ExampleSubsystem. */
  public mover() {
    elevatorController = new PIDController(ElevatorConstants.kp, ElevatorConstants.ki, ElevatorConstants.kd);
    elevatorController.setSetpoint(0);
    LeftElevatorMotor.setIdleMode(IdleMode.kBrake);
    RightElevatorMotor.setIdleMode(IdleMode.kBrake);
    RightElevatorMotor.follow(LeftElevatorMotor, true);

    armController = new PIDController(ArmConstants.kp, ArmConstants.ki, ArmConstants.kd);
    armController.setSetpoint(0);
    armMotor.setIdleMode(IdleMode.kBrake);
  }

  public void move(double[] speeds) {
    LeftElevatorMotor.set(speeds[1]);
    armMotor.set(speeds[0]);
  }

  public Pose2d getPos() {
    return new Pose2d(armEncoder.getPosition(), elevatorEncoder.getPosition(), new Rotation2d());
  }

  public Command getMovements(Pose2d endPos, double speed) {
    SmartDashboard.putNumber("speed thing", speed);
    double[] upDistance = {0, Math.max(ElevatorConstants.bumperRots - getPos().getY(), 0), speed};
    double[] distances = {endPos.getX() - getPos().getX(), endPos.getY() - getPos().getY(), speed};

    return Commands.sequence(new moveConMover(this, upDistance), new moveConMover(this, distances));
  }

  public double[] calculate(double[] calculations) {
    double[] output = {armController.calculate(calculations[0]), elevatorController.calculate(calculations[1])};
    return output;
  }

  public void setSetpoints(double[] setpoints) {
    armController.setSetpoint(setpoints[0]);
    elevatorController.setSetpoint(setpoints[1]);
  }

  public boolean[] atSetpoint() {
    boolean[] output = {armController.atSetpoint(), elevatorController.atSetpoint()};
    return output;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("elevator encoder", elevatorEncoder.getPosition());
    SmartDashboard.putNumber("arm encoder", armEncoder.getPosition());
  }
}
