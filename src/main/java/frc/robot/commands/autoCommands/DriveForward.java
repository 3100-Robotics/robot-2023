package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain;
import frc.robot.Constants.driveTrainConstants;;

public class DriveForward extends CommandBase{
    
    drivetrain Drive;
    double speed, distance;

    public DriveForward(drivetrain Drive, double speed, double distance) {
        this.Drive = Drive;
        this.speed = speed;
        this.distance = distance * driveTrainConstants.encoderScale;
        
        addRequirements(Drive);
    }

    public void initialize() {
        Drive.resetEncoders();
    }

    public void execute() {
        Drive.arcadeDrive(speed, 0.0);
    }

    public boolean isFinished() {
        if (speed < 0 && -Drive.getAverageEncoderRotation() >= distance) {
            System.out.println("done driving");
            return true;
        }
        else if (Drive.getAverageEncoderRotation() >= distance) { 
            System.out.println("done driving");
            return true;
        }
        return false;
    }
}
