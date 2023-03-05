// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.IOConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.driving;
import frc.robot.commands.autoCommands.PIDBallence;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.drivetrain;
import frc.robot.subsystems.endAffector;
// import frc.robot.subsystems.vision;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import edu.wpi.first.wpilibj2.command.button.Trigger;

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
  // EventLoop povLoop = new EventLoop();

  // buttons for commands
  private JoystickButton driverButtonStart = new JoystickButton(m_driverController, IOConstants.startButtonChannel);
  private JoystickButton driverButtonSelect = new JoystickButton(m_driverController, IOConstants.backButtonChannel);
  private JoystickButton driverButtonB = new JoystickButton(m_driverController, IOConstants.bButtonChannel);
  private JoystickButton driverRightBumper = new JoystickButton(m_driverController, IOConstants.rightBumperChannel);
  private JoystickButton buttonX = new JoystickButton(m_codriverController, IOConstants.xButtonChannel);
  private JoystickButton buttonY = new JoystickButton(m_codriverController, IOConstants.yButtonChannel);
  private JoystickButton buttonA = new JoystickButton(m_codriverController, IOConstants.aButtonChannel);
  private JoystickButton buttonB = new JoystickButton(m_codriverController, IOConstants.bButtonChannel);
  // private JoystickButton buttonStart = new JoystickButton(m_codriverController, IOConstants.startButtonChannel);
  private JoystickButton buttonSelect = new JoystickButton(m_codriverController, IOConstants.backButtonChannel);
  private JoystickButton buttonlb = new JoystickButton(m_codriverController, IOConstants.leftBumperChannel);
  private JoystickButton buttonrb = new JoystickButton(m_codriverController, IOConstants.rightBumperChannel);
  // private Trigger buttonDUp = new Trigger(m_codriverController.pov(IOConstants.POVU, povLoop));
  // private Trigger buttonDDown = new Trigger(m_codriverController.pov(IOConstants.POVD, povLoop));

  // subsystems
  private final drivetrain drive = new drivetrain();
  private final Arm arm = new Arm();
  private final Elevator elevator = new Elevator();
  private final endAffector claw = new endAffector();
  private final PIDBallence autoballence = new PIDBallence(drive);
  // private final vision m_Vision = new vision();

  private final Command m_noAuto = null;

  private final Command driveOutShortRight = Autos.drive(drive, 0.3, Units.metersToFeet(2));
  private final Command driveOutLongRight = Autos.drive(drive, 0.3, Units.metersToFeet(4));
  private final Command driveOutShortLeft = Autos.drive(drive, -0.3, Units.metersToFeet(2));
  private final Command driveOutLongLeft = Autos.drive(drive, 0.3, Units.metersToFeet(4));

  // A complex auto routine that drives forward, drops a hatch, and then drives backward.
  private final Command m_ballence = Autos.ballence(drive, 0.3, Units.metersToFeet(1.5));
  private final Command m_scoreCube = Autos.scoreCubeStay(elevator, arm, claw);
  private final Command m_scoreCubeLeave = Autos.scoreCubeLeave(drive, elevator, arm, claw, -0.3, 6);

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
    arm.setDefaultCommand(new ArmCommand(arm, m_codriverController));
    elevator.setDefaultCommand(new ElevatorCommand(elevator, m_codriverController));
    // claw.setDefaultCommand(new endAffectorController(m_codriverController, claw));
    // m_Vision.setDefaultCommand(new visionController(m_codriverController, m_Vision, claw));

    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    // drivetrain commands
    driverRightBumper.onTrue(new InstantCommand(() -> drive.ToggleSlowMode(), drive));
    // driverRightBumper.whileTrue(new StartEndCommand(() -> drive.setSlowMode(false), () -> drive.setSlowMode(true), drive));
    driverButtonB.whileTrue(autoballence);
    
    // end affector commands

    // lock claw joystick movements
    buttonSelect.onTrue(new InstantCommand(() -> claw.toggleEndAffectorLock(), claw));

    // run both the claws at once
    buttonlb.whileTrue(new StartEndCommand(
      () -> claw.runBoth(0.2),
      () -> claw.stopBoth(),
    claw));
    
    buttonrb.whileTrue(new StartEndCommand(
      () -> claw.runBoth(-0.2),
      () -> claw.stopBoth(),
    claw));

    buttonA.whileTrue(new StartEndCommand(
      () -> claw.runLeft(-0.2), 
      () -> claw.stopLeft(), 
    claw));

    buttonB.whileTrue(new StartEndCommand(
      () -> claw.runLeft(0.2), 
      () -> claw.stopLeft(), 
    claw));

    buttonX.whileTrue(new StartEndCommand(
      () -> claw.runRight(-0.2), 
      () -> claw.stopRight(), 
    claw));

    buttonY.whileTrue(new StartEndCommand(
      () -> claw.runRight(0.2), 
      () -> claw.stopRight(), 
    claw));
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_chooser.getSelected();
  }
}
