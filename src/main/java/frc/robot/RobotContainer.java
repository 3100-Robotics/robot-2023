// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.IOConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.driveTrainConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.RunElevator;
import frc.robot.commands.driving;
import frc.robot.commands.autoCommands.PIDBallence;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.drivetrain;
import frc.robot.subsystems.endAffector;

import java.io.IOException;
import java.nio.file.Path;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

// import frc.robot.subsystems.vision;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
// import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // contrillers
  private final XboxController m_driverController =
      new XboxController(OperatorConstants.kDriverControllerPort);
  private final XboxController m_codriverController =
      new XboxController(OperatorConstants.kCoDriverControllerPort);

  // event loop neeed to use dpad (Pretty sure this is actually neeed)
  EventLoop povLoop = new EventLoop();

  // buttons for commands
  private JoystickButton driverButtonStart = new JoystickButton(m_driverController, IOConstants.startButtonChannel);
  private JoystickButton driverButtonSelect = new JoystickButton(m_driverController, IOConstants.backButtonChannel);
  private JoystickButton driverButtonB = new JoystickButton(m_driverController, IOConstants.bButtonChannel);
  private JoystickButton driverRightBumper = new JoystickButton(m_driverController, IOConstants.rightBumperChannel);
  private JoystickButton buttonX = new JoystickButton(m_codriverController, IOConstants.xButtonChannel);
  private JoystickButton buttonY = new JoystickButton(m_codriverController, IOConstants.yButtonChannel);
  private JoystickButton buttonA = new JoystickButton(m_codriverController, IOConstants.aButtonChannel);
  private JoystickButton buttonB = new JoystickButton(m_codriverController, IOConstants.bButtonChannel);
  private JoystickButton buttonStart = new JoystickButton(m_codriverController, IOConstants.startButtonChannel);
  private JoystickButton buttonSelect = new JoystickButton(m_codriverController, IOConstants.backButtonChannel);
  private JoystickButton buttonLeftBumper = new JoystickButton(m_codriverController, IOConstants.leftBumperChannel);
  private JoystickButton buttonRightBumper = new JoystickButton(m_codriverController, IOConstants.rightBumperChannel);
  private Trigger buttonDUp = new Trigger(m_codriverController.pov(IOConstants.POVU, povLoop));
  private Trigger buttonDDown = new Trigger(m_codriverController.pov(IOConstants.POVD, povLoop));

  // subsystems
  private final drivetrain drive = new drivetrain();
  private final Arm arm = new Arm();
  private final Elevator elevator = new Elevator();
  private final endAffector claw = new endAffector();
  private final PIDBallence autoballence = new PIDBallence(drive);
  // private final vision m_Vision = new vision();

  // no auto
  private final Command m_noAuto = null;

  // driving out
  private final Command driveOutShortRight = Autos.drive(drive, 0.3, Units.metersToFeet(2));
  private final Command driveOutLongRight = Autos.drive(drive, 0.3, Units.metersToFeet(4));
  private final Command driveOutShortLeft = Autos.drive(drive, -0.3, Units.metersToFeet(2));
  private final Command driveOutLongLeft = Autos.drive(drive, 0.3, Units.metersToFeet(4));

  // fancy autos
  private final Command m_ballence = Autos.ballence(drive, 0.3, Units.metersToFeet(1.5));
  private final Command m_scoreCube = Autos.scoreCubeStay(elevator, arm, claw);
  private final Command m_scoreCubeLeave = Autos.scoreCubeLeave(drive, elevator, arm, claw, -0.3, 6);

  // path planner setup

  private final Command m_PathPlannerCommand = null;
  private final PathPlannerTrajectory m_testPathPlanner = PathPlanner.loadPath("double score", new PathConstraints(4, 3));


  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // get them cameras
    CameraServer.startAutomaticCapture();

    m_chooser.setDefaultOption("No Auto", m_noAuto);
    m_chooser.addOption("drive out right charge", driveOutShortRight);
    m_chooser.addOption("drive out long right", driveOutLongRight);
    m_chooser.addOption("drive out left short", driveOutShortLeft);
    m_chooser.addOption("drive out left long", driveOutLongLeft);
    m_chooser.addOption("ballence", m_ballence);
    m_chooser.addOption("score cube", m_scoreCube);
    m_chooser.addOption("score cube leave", m_scoreCubeLeave);

    SmartDashboard.putData(m_chooser);

    // default commands
    drive.setDefaultCommand(new driving(drive, m_driverController));
    elevator.setDefaultCommand(new RunElevator(elevator, 0.4));
    // m_Vision.setDefaultCommand(new visionController(m_codriverController, m_Vision, claw));

    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    // drivetrain commands
    driverRightBumper.onTrue(new InstantCommand(() -> drive.ToggleSlowMode(), drive));
    driverButtonB.whileTrue(autoballence);
    
    // end affector commands

    // lock claw joystick movements
    buttonSelect.onTrue(new InstantCommand(() -> claw.toggleEndAffectorLock(), claw));

    // movement commands

    buttonY.onTrue(new InstantCommand(() -> elevator.position = "high", elevator));
    buttonA.onTrue(new InstantCommand(() -> elevator.position = "ground", elevator));
    buttonX.onTrue(new InstantCommand(() -> elevator.position = "mid", elevator));
    buttonB.onTrue(new InstantCommand(() -> elevator.position = "player", elevator));
    buttonRightBumper.onTrue(new InstantCommand(() -> elevator.altPos = !elevator.altPos, elevator));

    buttonY.onTrue(new InstantCommand(() -> arm.position = "high", arm));
    buttonA.onTrue(new InstantCommand(() -> arm.position = "ground", arm));
    buttonX.onTrue(new InstantCommand(() -> arm.position = "mid", arm));
    buttonB.onTrue(new InstantCommand(() -> arm.position = "player", arm));
    buttonRightBumper.onTrue(new InstantCommand(() -> arm.altPos = !arm.altPos, arm));
  }

  public Command getAutonomousCommand() {

    return m_chooser.getSelected();
  }

  public Command loadPathplannerTrajectory(String filename, boolean resetOdometry){
    Trajectory trajectory;

    try{
      // get the directory name where the trajectory path is located
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filename);
      // build the trajectory from PathweaverJson file
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    }
    catch(IOException exception){
      DriverStation.reportError("unableto open trajectory file "+filename, exception.getStackTrace());

      System.out.println("Unable to read from file"+filename );

      return new InstantCommand();  // do nothing command
    }

    // // Construct command to follow trajectory
    // RamseteCommand command =
    //     new RamseteCommand(
    //         trajectory,
    //         drive::getPose,
    //         DriveConstants.kFeedforward,
    //         driveTrainConstants.kDriveKinematics,

    //         // Position controllers
    //         new PIDController(AutoConstants.kPXController, 0, 0),
    //         new PIDController(AutoConstants.kPYController, 0, 0),
    //         new ProfiledPIDController(
    //             AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints),

    //         // Needed for normalizing wheel speeds
    //         driveTrainConstants.kMaxSpeedMetersPerSecond,

    //         // Velocity PID's
    //         new PIDController(DriveConstants.kPFrontLeftVel, 0, 0),
    //         new PIDController(DriveConstants.kPRearLeftVel, 0, 0),
    //         new PIDController(DriveConstants.kPFrontRightVel, 0, 0),
    //         new PIDController(DriveConstants.kPRearRightVel, 0, 0),
    //         m_DriveSubsystem::getCurrentWheelSpeeds,
    //         m_DriveSubsystem::setDriveMotorControllersVolts, // Consumer for the output motor voltages
    //         drive);

    if (resetOdometry){
      // Reset odometry to the starting pose of the trajectory.
      drive.resetOdometry(trajectory.getInitialPose());
    }

    // // Run path following command, then stop at the end.
    // return mecanumControllerCommand.andThen(() -> m_DriveSubsystem.drive(0, 0, 0));
    return null;
  }
}
