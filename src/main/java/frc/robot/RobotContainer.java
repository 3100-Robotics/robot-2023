// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.driving;
import frc.robot.commands.moveClaw;
import frc.robot.commands.autoCommands.PIDBallence;
import frc.robot.subsystems.drivetrain;
import frc.robot.subsystems.endAffector;
import frc.robot.subsystems.mover;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // contrillers
  private final XboxController driverController =
      new XboxController(OperatorConstants.kDriverControllerPort);
  private final XboxController codriverController =
      new XboxController(OperatorConstants.kCoDriverControllerPort);

  // event loop neeed to use dpad (Pretty sure this is actually neeed)
  EventLoop povLoop = new EventLoop();

  // buttons for commands
  private final JoystickButton driverButtonB = new JoystickButton(driverController, IOConstants.bButtonChannel);
  private final JoystickButton driverRightBumper = new JoystickButton(driverController, IOConstants.rightBumperChannel);
  private final JoystickButton buttonX = new JoystickButton(codriverController, IOConstants.xButtonChannel);
  private final JoystickButton buttonY = new JoystickButton(codriverController, IOConstants.yButtonChannel);
  private final JoystickButton buttonA = new JoystickButton(codriverController, IOConstants.aButtonChannel);
  private final JoystickButton buttonB = new JoystickButton(codriverController, IOConstants.bButtonChannel);
  private final JoystickButton buttonSelect = new JoystickButton(codriverController, IOConstants.backButtonChannel);
  private final JoystickButton buttonLeftBumper = new JoystickButton(codriverController, IOConstants.leftBumperChannel);
  private final JoystickButton buttonRightBumper = new JoystickButton(codriverController, IOConstants.rightBumperChannel);
  private final Trigger buttonDUp = new Trigger(codriverController.pov(IOConstants.POVU, povLoop));
  private final Trigger buttonDDown = new Trigger(codriverController.pov(IOConstants.POVD, povLoop));
  private final Trigger buttonDLeft = new Trigger(codriverController.pov(IOConstants.POVL, povLoop));
  private final Trigger buttonDRight = new Trigger(codriverController.pov(IOConstants.POVR, povLoop));

  // subsystems
  private final drivetrain drive = new drivetrain();
  private final endAffector claw = new endAffector();
  private final mover elearm = new mover();
  // private final vision m_Vision = new vision();

  ///////////
  // AUTOS //
  ///////////

  private final Command noAuto = null;

  private final Command driveOutRightLong = Autos.drive(drive, 0.3, Units.metersToFeet(4.5));
  private final Command driveOutRightShort = Autos.drive(drive, 0.3, Units.metersToFeet(2));
  private final Command driveOutLeft = Autos.drive(drive, 0.3, Units.metersToFeet(3));

  private final Command ScoreBallance = Commands.sequence(new InstantCommand(() -> elearm.getMovements(new Pose2d(ArmConstants.highLength, ElevatorConstants.highHeight, new Rotation2d(0))), elearm),
    Autos.ballence(drive, -0.3, Units.metersToFeet(1.5)));
  private final Command ScoreLeaveRight = Commands.sequence(new InstantCommand(() -> elearm.getMovements(new Pose2d(ArmConstants.highLength, ElevatorConstants.highHeight, new Rotation2d(0))), elearm),
    driveOutRightLong);
  private final Command ScoreLeaveLeft = Commands.sequence(new InstantCommand(() -> elearm.getMovements(new Pose2d(ArmConstants.highLength, ElevatorConstants.highHeight, new Rotation2d(0))), elearm),
    driveOutLeft);

  private final Command m_ballance = Autos.ballence(drive, 0.3, Units.metersToFeet(1.5));

  SendableChooser<Command> chooser = new SendableChooser<>();

  public RobotContainer() {
    // get them cameras
    CameraServer.startAutomaticCapture();

    chooser.setDefaultOption("No Auto", noAuto);
    chooser.addOption("driveOutRightLong", driveOutRightLong);
    chooser.addOption("driveOutRightShort", driveOutRightShort);
    chooser.addOption("driveOutLeft", driveOutLeft);
    chooser.addOption("ballance", m_ballance);
    chooser.addOption("ScoreBallance", ScoreBallance);
    chooser.addOption("ScoreLeaveRight", ScoreLeaveRight);
    chooser.addOption("ScoreLeaveLeft", ScoreLeaveLeft);

    SmartDashboard.putData(chooser);

    // default commands
    drive.setDefaultCommand(new driving(drive, driverController));
    claw.setDefaultCommand(new moveClaw(codriverController, claw));
    elearm.setDefaultCommand(new InstantCommand(() -> elearm.move(new double[]{0.02, 0.02}), elearm));
    // m_Vision.setDefaultCommand(new visionController(codriverController, m_Vision, claw));

    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    // drivetrain commands
    driverRightBumper.onTrue(new InstantCommand(drive::ToggleSlowMode, drive));
    driverButtonB.whileTrue(new PIDBallence(drive));
    
    // end affector commands

    // lock claw joystick movements
    buttonSelect.onTrue(new InstantCommand(claw::toggleEndAffectorLock, claw));

    // movement commands

    buttonLeftBumper.whileTrue(new StartEndCommand(
      () -> claw.runBoth(0.2),
            claw::stopBoth,
    claw));
    
    buttonRightBumper.whileTrue(new StartEndCommand(
      () -> claw.runBoth(-0.2),
            claw::stopBoth,
    claw));

//    buttonLeftBumper.onTrue(new InstantCommand(() -> elearm.getMovements(new Pose2d(0, 0, new Rotation2d())), elearm));
    buttonY.onTrue(new InstantCommand(() -> elearm.getMovements(new Pose2d(ArmConstants.highLength, ElevatorConstants.highHeight, new Rotation2d(0))), elearm));
    buttonA.onTrue(new InstantCommand(() -> elearm.getMovements(new Pose2d(ArmConstants.bumperRots, ElevatorConstants.floorHeight, new Rotation2d(0))), elearm));
    buttonX.onTrue(new InstantCommand(() -> elearm.getMovements(new Pose2d(ArmConstants.midLength, ElevatorConstants.midHeight, new Rotation2d(0))), elearm));
    buttonB.onTrue(new InstantCommand(() -> elearm.getMovements(new Pose2d(0, 0, new Rotation2d(0))), elearm));


    double[] goUp = {0, 0.3};
    double[] goDown = {0, -0.3};
    double[] goOut = {0.3, 0};
    double[] goIn = {-0.3, 0};
    double[] stop = {0.02, 0.02};
    buttonDUp.whileTrue(new StartEndCommand(() -> elearm.move(goUp), () -> elearm.move(stop), elearm));
    buttonDDown.whileTrue(new StartEndCommand(() -> elearm.move(goDown), () -> elearm.move(stop), elearm));
    buttonDRight.whileTrue(new StartEndCommand(() -> elearm.move(goOut), () -> elearm.move(stop), elearm));
    buttonDLeft.whileTrue(new StartEndCommand(() -> elearm.move(goIn), () -> elearm.move(stop), elearm));
  }

  public Command getAutonomousCommand() {

    return chooser.getSelected();
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
