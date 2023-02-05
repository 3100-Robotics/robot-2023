package frc.robot.commands.autoCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.driveTrainConstants;
import frc.robot.subsystems.drivetrain;

public class DriveTurn extends PIDCommand{
    drivetrain Drive;

    public DriveTurn(drivetrain Drive, double rotation) {
        super(new PIDController(driveTrainConstants.kp, driveTrainConstants.ki, driveTrainConstants.kd), 
        Drive::getgyroz, rotation, output -> Drive.arcadeDrive(0, output), Drive);
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
