// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.IOConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.ElevatorCommand;
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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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

  // buttons for commands
  private JoystickButton driverButtonB = new JoystickButton(m_driverController, IOConstants.bButtonChannel);
  private JoystickButton buttonSelect = new JoystickButton(m_codriverController, IOConstants.backButtonChannel);

  // subsystems
  private final Drive drive = new Drive();
  private final Arm arm = new Arm();
  private final Elevator elevator = new Elevator();
  private final Claw claw = new Claw();
  // private final vision m_Vision = new vision();

  ///////////
  // autos //
  ///////////

  private final Command m_noAuto = new InstantCommand();
  private final Command driveOutLong = Autos.drive(drive, 0.3, 15);
  private final Command driveOutMid = Autos.drive(drive, 0.3, 10);
  private final Command driveOutShort = Autos.drive(drive, 0.3, 6.5);
  private final Command m_balance = Autos.balance(drive, 0.3, 5);
  private final Command m_scoreCube = Autos.scoreCubeStay(elevator, arm, claw);
  private final Command m_scoreCubeLeave = Autos.scoreCubeLeave(drive, elevator, arm, claw, -0.3, 15);
  private final Command m_scoreCubeBalance = Autos.scoreCubeBalance(elevator, arm, claw, drive, 0.3, 10, 5);

  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // get them cameras
    CameraServer.startAutomaticCapture();

    m_chooser.setDefaultOption("score cube balance", m_scoreCubeBalance);
    m_chooser.addOption("balance", m_balance);
    m_chooser.addOption("score cube leave", m_scoreCubeLeave);
    m_chooser.addOption("score cube", m_scoreCube);
    m_chooser.addOption("drive out short", driveOutShort);
    m_chooser.addOption("drive out mid", driveOutMid);
    m_chooser.addOption("drive out long", driveOutLong);
    m_chooser.addOption("No Auto", m_noAuto);

    SmartDashboard.putData(m_chooser);

    // default commands
    drive.setDefaultCommand(new driveCommand(drive, elevator, m_driverController));
    arm.setDefaultCommand(new ArmCommand(arm, m_codriverController));
    elevator.setDefaultCommand(new ElevatorCommand(elevator, m_codriverController));
    claw.setDefaultCommand(new clawCommand(m_codriverController, claw));
    // m_Vision.setDefaultCommand(new visionController(m_codriverController, m_Vision, claw));

    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    // drivetrain commands
    driverButtonB.whileTrue(new balance(drive));
    
    // end affector commands

    // lock claw joystick movements
    buttonSelect.onTrue(new InstantCommand(() -> claw.toggleEndAffectorLock(), claw));

    // buttonlb.whileTrue(new StartEndCommand(
    //   () -> claw.runBoth(0.3),
    //   () -> claw.stopBoth(),
    // claw));
    
    // buttonrb.whileTrue(new StartEndCommand(
    //   () -> claw.runBoth(-0.3),
    //   () -> claw.stopBoth(),
    // claw));

    // buttonA.whileTrue(new StartEndCommand(
    //   () -> claw.runLeft(-0.3),
    //   () -> claw.stopLeft(), 
    // claw));

    // buttonB.whileTrue(new StartEndCommand(
    //   () -> claw.runLeft(0.3),
    //   () -> claw.stopLeft(), 
    // claw));

    // buttonX.whileTrue(new StartEndCommand(
    //   () -> claw.runRight(-0.3),
    //   () -> claw.stopRight(), 
    // claw));

    // buttonY.whileTrue(new StartEndCommand(
    //   () -> claw.runRight(0.3),
    //   () -> claw.stopRight(), 
    // claw));
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_chooser.getSelected();
  }
}
