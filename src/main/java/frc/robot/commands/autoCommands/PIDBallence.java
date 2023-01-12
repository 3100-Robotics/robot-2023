package frc.robot.commands.autoCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.driveTrainConstants;
import frc.robot.subsystems.drivetrain;

public class PIDBallence extends PIDCommand{
    drivetrain Drive;

    public PIDBallence(drivetrain Drive, double speed) {
        super(new PIDController(driveTrainConstants.kp, driveTrainConstants.ki, driveTrainConstants.kd), 
        Drive::getgyroy, 0, ouput -> Drive.arcadeDrive(speed, 0), Drive);
        this.Drive = Drive;
        getController().enableContinuousInput(-180, 180);
        // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
        // setpoint before it is considered as having reached the reference
        getController()
            .setTolerance(driveTrainConstants.kDriveToleranceDeg, driveTrainConstants.kDriveRateToleranceMeterPerS);
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
