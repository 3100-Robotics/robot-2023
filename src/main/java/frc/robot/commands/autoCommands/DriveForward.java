package frc.robot.commands.autoCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.drivetrain;
import frc.robot.Constants.driveTrainConstants;;

public class DriveForward extends PIDCommand{
    
    drivetrain Drive;
    double speed, distance;

    public DriveForward(drivetrain Drive, double distance) {
        super(new PIDController(driveTrainConstants.kp, driveTrainConstants.ki, driveTrainConstants.kd), 
        Drive::getAverageEncoderRotation, distance, output -> Drive.arcadeDrive(output, 0), Drive);
        this.Drive = Drive;
        getController().enableContinuousInput(-180, 180);
        // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
        // setpoint before it is considered as having reached the reference
        getController().setTolerance(driveTrainConstants.kDriveToleranceMeter,
            driveTrainConstants.kDriveRateToleranceMeterPerS);
    }

    @Override
    public void initialize() {
        Drive.resetEncoders();
    }

    @Override
    public boolean isFinished() {
      // End when the controller is at the reference.
      return getController().atSetpoint();
    }
}
