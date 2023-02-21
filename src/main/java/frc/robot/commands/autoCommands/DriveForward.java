package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.driveTrainConstants;
import frc.robot.subsystems.drivetrain;

public class DriveForward extends CommandBase{
    
    drivetrain Drive;
    double distance;

    public DriveForward(drivetrain Drive, double distance) {
        this.Drive = Drive;
        this.distance = distance;
    }

    @Override
    public void initialize() {
        Drive.setSetpoint(Drive.getAverageEncoderRotation() + distance * driveTrainConstants.tick2Feet);
    }

    @Override
    public void execute() {
        Drive.arcadeDrive(Drive.driveCalculate(Drive.getAverageEncoderRotation()), 0);
    }

    @Override
    public boolean isFinished() {
      // End when the controller is at the reference.
      return Drive.atSetpoint();
    }
}
