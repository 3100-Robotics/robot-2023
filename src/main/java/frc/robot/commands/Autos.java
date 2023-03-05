// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.commands.autoCommands.MoveArm;
import frc.robot.commands.autoCommands.MoveElevator;
import frc.robot.commands.autoCommands.PIDBallence;
import frc.robot.commands.autoCommands.openAffector;
import frc.robot.commands.autoCommands.simpleDrive;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.drivetrain;
import frc.robot.subsystems.endAffector;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {

  // drive forward
  public static CommandBase drive(drivetrain subsystem, double speed, double distance) {
    return Commands.sequence(new simpleDrive(subsystem, speed, distance));
  }

  public static CommandBase ballence(drivetrain drive, double speed, double backDistance) {
    return Commands.sequence(new simpleDrive(drive, speed, backDistance), new PIDBallence(drive));
  }

  public static CommandBase scoreCubeStay(Elevator elevator, Arm arm, endAffector end) {
    return Commands.sequence(new MoveElevator(elevator, 0.4, 2.3*12*7), new MoveArm(arm, 0.3, 1.7*12*6.5), new openAffector(end, 0.3, 0.025*3));
  }

  public static CommandBase scoreCubeLeave(drivetrain drive, Elevator elevator, Arm arm, endAffector end,
                  double speed, double distance) {
      return Commands.sequence(scoreCubeStay(elevator, arm, end), drive(drive, speed, distance));
    }

  private Autos() {
    // not meant to be defiend
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
