package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.moveConMover;

public class mover extends SubsystemBase {

  // elevator
  private final CANSparkMax LeftElevatorMotor = new CANSparkMax(ElevatorConstants.LeftElevatorMotor, MotorType.kBrushless);
  private final CANSparkMax RightElevatorMotor = new CANSparkMax(ElevatorConstants.RightElevatorMotor, MotorType.kBrushless);
  
  private final CANSparkMax armMotor = new CANSparkMax(ArmConstants.armMotor, MotorType.kBrushless);

  private final RelativeEncoder elevatorEncoder = LeftElevatorMotor.getEncoder();
  private final RelativeEncoder armEncoder = armMotor.getEncoder();

  // private SlewRateLimiter limiter = new SlewRateLimiter(ElevatorConstants.slewRate);
  // private SlewRateLimiter limiter = new SlewRateLimiter(ArmMotorConstants.slewRate);

  public PIDController  elevatorController;
  public PIDController  armController;

  /** Creates a new ExampleSubsystem. */
  public mover() {
    elevatorController = new PIDController(ElevatorConstants.kp, ElevatorConstants.ki, ElevatorConstants.kd);
    elevatorController.setSetpoint(0);
    // elevatorController = LeftElevatorMotor.getPIDController();
    // elevatorController.setP(ElevatorConstants.kp);
    // elevatorController.setI(ElevatorConstants.ki);
    // elevatorController.setD(ElevatorConstants.kd);
    // elevatorController.setIZone(0);

    LeftElevatorMotor.setIdleMode(IdleMode.kBrake);
    RightElevatorMotor.setIdleMode(IdleMode.kBrake);
    RightElevatorMotor.follow(LeftElevatorMotor, true);

    armController = new PIDController(ArmConstants.kp, ArmConstants.ki, ArmConstants.kd);
    armController.setSetpoint(0);
    // armController = armMotor.getPIDController();
    // armController.setP(ArmConstants.kp);
    // armController.setI(ArmConstants.ki);
    // armController.setD(ArmConstants.kd);
    // armController.setIZone(0);

    armMotor.setIdleMode(IdleMode.kBrake);
    armMotor.setInverted(false);
  }

  public void move(double[] speeds) {
    LeftElevatorMotor.set(speeds[1]);
    armMotor.set(speeds[0]);
  }

  public Pose2d getPos() {
    return new Pose2d(armEncoder.getPosition(), elevatorEncoder.getPosition(), new Rotation2d());
  }

  public void getMovements(Pose2d endPos) {
    double[] upDistance = {0, ElevatorConstants.bumperRots};
    double[] distances = {endPos.getX(), endPos.getY()};

    Command command = Commands.sequence(
            ((getPos().getY() < ElevatorConstants.bumperRots) ?
                    new moveConMover(this, upDistance): new InstantCommand(() -> System.out.println("do nothing"))),//);
//            ((getPos().getX() > ArmConstants.playerLength && getPos().getX() < ArmConstants.midLength) ?
//                    new moveConMover(this, new double[]{0, getPos().getY()}): new InstantCommand()),
    (new moveConMover(this, distances)));
    command.schedule();
  }

  public double[] calculate(double[] calculations) {
    return new double[]{armController.calculate(calculations[0]), elevatorController.calculate(calculations[1])};
  }

  public void setSetpoints(double[] setpoints) {
    armController.setSetpoint(setpoints[0]);
    elevatorController.setSetpoint(setpoints[1]);
  }

  public boolean atSetpoint() {
    return (armController.atSetpoint() && elevatorController.atSetpoint());
  }

  public void doSmartMotion(double[] wantedPos) {
    // elevatorController.setReference(wantedPos[1], CANSparkMax.ControlType.kSmartMotion);
    // armController.setReference(wantedPos[0], CANSparkMax.ControlType.kSmartMotion);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("elevator encoder", elevatorEncoder.getPosition());
    SmartDashboard.putNumber("arm encoder", armEncoder.getPosition());
  }
}
