// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.commands.autoCommands.DriveForward;
import frc.robot.commands.autoCommands.PIDBallence;
import frc.robot.subsystems.drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {

  // drive forward
  public static CommandBase drive(drivetrain subsystem, double speed, double distance) {
    return Commands.sequence(new DriveForward(subsystem, speed, distance));
  }

  public static CommandBase ballence(drivetrain drive, double speed, double backDistance) {
    return Commands.sequence(new DriveForward(drive, speed, backDistance), new PIDBallence(drive));
  }

  private Autos() {
    // not meant to be defiend
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
