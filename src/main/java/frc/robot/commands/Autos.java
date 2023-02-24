// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.commands.autoCommands.DriveForward;
import frc.robot.commands.autoCommands.MoveArm;
import frc.robot.commands.autoCommands.MoveElevator;
import frc.robot.commands.autoCommands.PIDBallence;
import frc.robot.commands.autoCommands.openAffector;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.drivetrain;
import frc.robot.subsystems.endAffector;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static CommandBase driveForward(drivetrain subsystem, double distance) {
    return Commands.sequence(new DriveForward(subsystem, distance));
  }

  public static CommandBase score(Elevator elevator, Arm arm, endAffector affector) {
    return Commands.sequence(new MoveElevator(elevator, false), new MoveElevator(elevator, false),
                             new MoveArm(arm, false), new MoveArm(arm, false), new openAffector(affector, 1));
  }


  public static CommandBase scoreleave(drivetrain drive, Arm arm, Elevator elevator, endAffector affector){
    return Commands.sequence(new MoveElevator(elevator, false), new MoveElevator(elevator, false),
                            new MoveArm(arm, false), new MoveArm(arm, false),
                            new openAffector(affector, 1), new DriveForward(drive, -5));
  }

  public static CommandBase scoreballance(drivetrain drive, Arm arm, Elevator elevator, endAffector ea, PIDBallence balance) {
    return Commands.sequence(new MoveElevator(elevator, false), new MoveElevator(elevator, false),
                            new MoveArm(arm, false), new MoveArm(arm, false),
                            new openAffector(ea, 1),
                            new DriveForward(drive, -5),
                            new MoveElevator(elevator, true), new MoveElevator(elevator, true),
                            new MoveArm(arm, true), new MoveArm(arm, true),
                            new PIDBallence(drive));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
