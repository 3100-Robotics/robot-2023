// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.IOConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.driving;
import frc.robot.commands.endAffectorController;
import frc.robot.commands.visionController;
import frc.robot.commands.autoCommands.PIDBallence;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.drivetrain;
import frc.robot.subsystems.endAffector;
import frc.robot.subsystems.vision;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
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
  private final XboxController m_driverController =
      new XboxController(OperatorConstants.kDriverControllerPort);
  private final XboxController m_codriverController =
      new XboxController(OperatorConstants.kCoDriverControllerPort);

  // event loop neeed to use dpad (Pretty sure this is actually neeed)
  EventLoop povLoop = new EventLoop();

  // buttons for commands
  private JoystickButton driverButtonStart = new JoystickButton(m_driverController, IOConstants.startButtonChannel);
  private JoystickButton driverButtonSelect = new JoystickButton(m_driverController, IOConstants.backButtonChannel);
  private JoystickButton buttonX = new JoystickButton(m_codriverController, IOConstants.xButtonChannel);
  private JoystickButton buttonY = new JoystickButton(m_codriverController, IOConstants.yButtonChannel);
  private JoystickButton buttonA = new JoystickButton(m_codriverController, IOConstants.aButtonChannel);
  private JoystickButton buttonB = new JoystickButton(m_codriverController, IOConstants.bButtonChannel);
  private JoystickButton buttonStart = new JoystickButton(m_codriverController, IOConstants.startButtonChannel);
  private JoystickButton buttonSelect = new JoystickButton(m_codriverController, IOConstants.backButtonChannel);
  private JoystickButton buttonlb = new JoystickButton(m_codriverController, IOConstants.leftBumperChannel);
  private JoystickButton buttonrb = new JoystickButton(m_codriverController, IOConstants.rightBumperChannel);
  private Trigger buttonDUp = new Trigger(m_codriverController.pov(IOConstants.POVU, povLoop));
  private Trigger buttonDDown = new Trigger(m_codriverController.pov(IOConstants.POVD, povLoop));

  // subsystems
  private final drivetrain drive = new drivetrain();
  private final Arm arm = new Arm();
  private final Elevator elevator = new Elevator();
  private final endAffector claw = new endAffector();
  private final PIDBallence autoballence = new PIDBallence(drive);
  private final vision m_Vision = new vision();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // get them cameras
    CameraServer.startAutomaticCapture();

    // default commands
    drive.setDefaultCommand(new driving(drive, m_driverController));
    arm.setDefaultCommand(new ArmCommand(arm));
    elevator.setDefaultCommand(new ElevatorCommand(elevator));
    claw.setDefaultCommand(new endAffectorController(m_codriverController, claw));
    // m_Vision.setDefaultCommand(new visionController(m_codriverController, m_Vision, claw));

    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    // drivetrain commands
    driverButtonStart.onTrue(new InstantCommand(() -> drive.ToggleSlowMode(), drive));
    driverButtonSelect.whileTrue(autoballence);

    // elevator commands

    // too fancy enable if I have time
    // buttonY.onTrue(new InstantCommand(
    //   () -> elevator.incrementSetpoint(false),
    // elevator));

    // buttonA.onTrue(new InstantCommand(
    //   () -> elevator.incrementSetpoint(true),
    // elevator));

    buttonY.whileTrue(new StartEndCommand(
      () -> elevator.Run(0.3), 
      () -> elevator.setSetpoint(elevator.GetEncoderRotation()),
    elevator));

    buttonA.whileTrue(new StartEndCommand(
      () -> elevator.Run(-0.1), 
      () -> elevator.setSetpoint(elevator.GetEncoderRotation()), 
    elevator));

    // arm commands

    // too fancy enable if I have time
    // buttonB.onTrue(new InstantCommand(
    //   () -> arm.incrementSetpoint(false),
    // arm));

    // buttonX.onTrue(new InstantCommand(
    //   () -> arm.incrementSetpoint(true),
    // arm));

    // buttonB.whileTrue(new StartEndCommand(
    //   () -> arm.Run(0.25), 
    //   () -> arm.setSetpoint(arm.GetEncoderRotation()), 
    // arm));

    // buttonX.whileTrue(new StartEndCommand(
    //   () -> arm.Run(-0.1), 
    //   () -> arm.setSetpoint(arm.GetEncoderRotation()), 
    // arm));

    buttonB.whileTrue(new StartEndCommand(
      () -> arm.Run(0.25), 
      () -> arm.Stop(), 
    arm));

    buttonX.whileTrue(new StartEndCommand(
      () -> arm.Run(-0.1), 
      () -> arm.Stop(), 
    arm));
    
    // end affector commands

    // lock claw joystick movements
    buttonSelect.onTrue(new InstantCommand(() -> claw.toggleEndAffectorLock(), claw));

    // run both the claws at once
    buttonlb.whileTrue(new StartEndCommand(
      () -> claw.runBoth(0.3),
      () -> claw.stopBoth(),
    claw));
    
    buttonrb.whileTrue(new StartEndCommand(
      () -> claw.runBoth(-0.3),
      () -> claw.stopBoth(),
    claw));
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
