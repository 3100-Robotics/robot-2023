package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.driveTrainConstants;
import frc.robot.subsystems.drivetrain;

public class DriveForward extends CommandBase{
    
    // var setup
    drivetrain Drive;
    double distance;

    public DriveForward(drivetrain Drive, double distance) {
        // typical stuff
        this.Drive = Drive;
        this.distance = distance;
        addRequirements(Drive);
        Drive.resetEncoders();
        Drive.setSetpoint(distance * driveTrainConstants.tick2feet);
        Drive.setSetpoint(distance);
    }

    @Override
    public void execute() {
        // move
        System.out.println("moving");
        Drive.arcadeDrive(Drive.driveCalculate(Drive.getAverageEncoderRotation()), 0);
    }

    @Override
    public boolean isFinished() {
      // Am I done?
      return Drive.atSetpoint();
    }
}
