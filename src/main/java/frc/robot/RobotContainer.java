// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
import com.pathplanner.lib.PathPlannerTrajectory.Waypoint;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.autoCommands.moveArmFancy;
import frc.robot.commands.autoCommands.moveClaw;
import frc.robot.commands.driveCommand;
import frc.robot.commands.clawCommand;
import frc.robot.commands.autoCommands.balance;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Claw;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import com.pathplanner.lib.commands.PPRamseteCommand;

import java.util.ArrayList;
import java.util.List;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // controllers
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController coDriverController =
      new CommandXboxController(OperatorConstants.CO_DRIVER_CONTROLLER_PORT);

  private final EventLoop dpadLoop = new EventLoop();


  // subsystems
  public final Drive drive = new Drive();
  public final Arm arm = new Arm();
  public final Elevator elevator = new Elevator();
  public final Claw claw = new Claw();
  // private final vision m_Vision = new vision();

  ///////////
  // autos //
  ///////////

  // A chooser for autonomous commands
  SendableChooser<Command> chooser = new SendableChooser<>();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // get them cameras
    CameraServer.startAutomaticCapture();

    Command m_scoreCubeBalance = Autos.scoreCubeBalance(elevator, arm, claw, drive, 0.3, 17.7, 7);
    Command m_balance = Autos.balance(drive, 0.3, 5);
    Command m_scoreCubeLeave = Autos.scoreCubeLeave(drive, elevator, arm, claw, -0.3, 20);
    Command m_scoreCube = Autos.scoreCubeStay(elevator, arm, claw);
    Command driveOutShort = Autos.drive(drive, 0.3, 6.5);
    Command driveOutMid = Autos.drive(drive, 0.3, 10);
    Command driveOutLong = Autos.drive(drive, 0.3, 15);
    Command m_noAuto = new InstantCommand();

    List<Waypoint> waypoints = new ArrayList<>();

//     List<EventMarker> eventMap = new ArrayList<>();
// //    HashMap<String, Command> eventMap = new HashMap<>();
// //    eventMap.put("scoreCubeHigh", Autos.scoreCubeStay(elevator, arm, claw));
// //    eventMap.put("scoreConeHigh", Autos.scoreCubeStay(elevator, arm, claw));
// //    eventMap.put("collect", Autos.collect(elevator, arm, claw));

//     Command testAuto = new PPRamseteCommand(new PathPlannerTrajectory(waypoints, eventMap,
//             new PathConstraints(3, 4), false, false),
//             drive::getPose, new RamseteController(),
//             Constants.driveTrainConstants.kinematics,
//             (aDouble, aDouble2) -> {}, drive);

//     chooser.setDefaultOption("score cube balance", m_scoreCubeBalance);
//     chooser.addOption("balance", m_balance);
//     chooser.addOption("score cube leave", m_scoreCubeLeave);
//     chooser.addOption("score cube", m_scoreCube);
//     chooser.addOption("drive out short", driveOutShort);
//     chooser.addOption("drive out mid", driveOutMid);
//     chooser.addOption("drive out long", driveOutLong);
//     chooser.addOption("No Auto", m_noAuto);

    SmartDashboard.putData(chooser);

    // default commands
    drive.setDefaultCommand(new driveCommand(drive, elevator, driverController));
    arm.setDefaultCommand(new ArmCommand(arm, coDriverController));
    elevator.setDefaultCommand(new ElevatorCommand(elevator, coDriverController));
    // claw.setDefaultCommand(new clawCommand(coDriverController, claw));
    // m_Vision.setDefaultCommand(new visionController(coDriverController, m_Vision, claw));

    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    // drivetrain commands
    // driverButtonB.whileTrue(new balance(drive));

    
    // end affector commands

    // lock claw joystick movements
    coDriverController.povUp().onTrue(new moveArmFancy(arm, -0.3, 2.5));
    coDriverController.povDown().onTrue(new moveClaw(claw, -0.3, 0));
    coDriverController.a().whileTrue(claw.runBothOut(0.4, 0));
    coDriverController.b().whileTrue(claw.runBothOut(-0.4, 0));
    

    coDriverController.leftBumper().whileTrue(new StartEndCommand(
      () -> claw.runBoth(0.4, 0.1),
      () -> claw.stopBoth(),
    claw));
    
    coDriverController.rightBumper().whileTrue(new StartEndCommand(
      () -> claw.runBoth(-0.4, 0.1),
      () -> claw.stopBoth(),
    claw));

    /*
     buttonA.whileTrue(new StartEndCommand(
       () -> claw.runLeft(-0.5),
       () -> claw.stopLeft(),
     claw));
     buttonB.whileTrue(new StartEndCommand(
       () -> claw.runLeft(0.5),
       () -> claw.stopLeft(),
     claw));
     buttonX.whileTrue(new StartEndCommand(
       () -> claw.runRight(-0.5),
       () -> claw.stopRight(),
     claw));
     buttonY.whileTrue(new StartEndCommand(
       () -> claw.runRight(0.5),
       () -> claw.stopRight(),
     claw));
    */

  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return chooser.getSelected();
  }
}
