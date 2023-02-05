// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.commands.autoCommands.MoveArm;
import frc.robot.commands.autoCommands.MoveElevator;
import frc.robot.commands.autoCommands.PIDBallence;
import frc.robot.commands.autoCommands.openAffector;
import frc.robot.subsystems.drivetrain;
import frc.robot.subsystems.endAffector;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static CommandBase DriveForward(drivetrain subsystem, double speed, double distance) {
    return Commands.sequence(DriveForward(subsystem, speed, distance));
  }

  public static CommandBase score(ElevatorCommand elevator, ArmCommand arm, endAffector affector) {
    return Commands.sequence(new MoveElevator(elevator, false), new MoveElevator(elevator, false),
                             new MoveArm(arm, false), new MoveArm(arm, false), new openAffector(affector, 1));
  }


  public static CommandBase scoreleave(drivetrain drive, ArmCommand arm, ElevatorCommand elevator, endAffector affector){
    return Commands.sequence(new MoveElevator(elevator, false), new MoveElevator(elevator, false),
                            new MoveArm(arm, false), new MoveArm(arm, false),
                            new openAffector(affector, 1), DriveForward(drive, -0.5, 5));
  }

  public static CommandBase scoreballance(drivetrain drive, ArmCommand arm, ElevatorCommand elevator, endAffector ea, PIDBallence balance) {
    return Commands.sequence(new MoveElevator(elevator, false), new MoveElevator(elevator, false),
                            new MoveArm(arm, false), new MoveArm(arm, false),
                            new openAffector(ea, 1), DriveForward(drive, -0.5, 5), new PIDBallence(drive));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
