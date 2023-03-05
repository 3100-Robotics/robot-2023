package frc.robot.commands.autoCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.driveTrainConstants;
import frc.robot.subsystems.drivetrain;

public class PIDBallence extends CommandBase{
    // make controller
    PIDController controller = new PIDController(
        driveTrainConstants.k_balenceP, driveTrainConstants.k_balenceI, driveTrainConstants.k_balenceD);

    // other vars
    drivetrain drive;
    double gyroReading, speed;

    public PIDBallence(drivetrain drive) {
        // typical stuff
        this.drive = drive;
        controller.setSetpoint(0);
        controller.setTolerance(0.3, 1);
        addRequirements(this.drive);
    }

    @Override
    public void execute() {
        // execute according to pid with speed limiting
        gyroReading = drive.getgyroy();
        speed = controller.calculate(gyroReading);
        if (Math.abs(speed) > 0.6) {
            speed = Math.copySign(0.6, speed);
        }
        drive.arcadeDrive(-speed, 0);
    }  

    @Override
    public boolean isFinished() {
        // am I finished?
        return false;
    }
    
}
