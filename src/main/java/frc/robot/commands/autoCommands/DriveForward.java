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
        Drive.setSetpoint(Drive.getAverageEncoderRotation() + distance * driveTrainConstants.tick2Feet);
    }

    @Override
    public void execute() {
        // move
        Drive.arcadeDrive(Drive.driveCalculate(Drive.getAverageEncoderRotation()), 0);
    }

    @Override
    public boolean isFinished() {
      // Am I done?
      return Drive.atSetpoint();
    }
}
