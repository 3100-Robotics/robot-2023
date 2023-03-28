// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.commands.autoCommands.moveForward;
import frc.robot.commands.autoCommands.moveArm;
import frc.robot.commands.autoCommands.MoveElevator;
import frc.robot.commands.autoCommands.balance;
import frc.robot.commands.autoCommands.moveClaw;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Claw;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {

  // drive forward
  public static CommandBase drive(Drive subsystem, double speed, double distance) {
    return Commands.sequence(new moveForward(subsystem, speed, distance));
  }


  public static CommandBase balance(Drive drive, double speed, double backDistance) {
    return Commands.sequence(
      drive(drive, speed, backDistance), 
      new balance(drive));
  }


  public static CommandBase scoreCubeStay(Elevator elevator, Arm arm, Claw end) {
    return Commands.sequence(
      new MoveElevator(elevator, 0.6, 83),
      new moveArm(arm, 0.6, 56), 
      new moveClaw(end, 0.6, 0.075),
            moveArmIn(elevator, arm));
  }

  public static CommandBase scoreCube(Elevator elevator, Arm arm, Claw end) {
    return Commands.sequence(
            new MoveElevator(elevator, 0.6, 83),
            new moveArm(arm, 0.6, 56),
            new moveClaw(end, 0.6, 0.075));
  }


  public static CommandBase moveArmIn(Elevator elevator, Arm arm) {
    return Commands.sequence(
      new moveArm(arm, -0.6, 1), 
      new MoveElevator(elevator, -0.6, 1));
  }
  

  public static CommandBase scoreCubeBalance(Elevator elevator, Arm arm, Claw end, Drive drive, 
      double driveSpeed, double backDistance, double forwardDistance) {
    return Commands.sequence(
      scoreCube(elevator, arm, end),
      moveArmIn(elevator, arm),
      drive(drive, -driveSpeed, backDistance), 
      balance(drive, driveSpeed, forwardDistance));
  }


  public static CommandBase scoreCubeLeave(Drive drive, Elevator elevator, Arm arm, Claw end,
        double speed, double distance) {
      return Commands.sequence(
        scoreCube(elevator, arm, end),
        moveArmIn(elevator, arm),
        drive(drive, speed, distance));
  }


  private Autos() {
    // not meant to be defined
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
