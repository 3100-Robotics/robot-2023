package frc.robot.commands.autoCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.driveTrainConstants;
import frc.robot.subsystems.drivetrain;

public class PIDBallence extends CommandBase{
    PIDController controller = new PIDController(driveTrainConstants.kp, driveTrainConstants.ki, driveTrainConstants.kd);
    drivetrain drive;
    double gyroReading, speed;

    public PIDBallence(drivetrain drive) {
        this.drive = drive;
        controller.setSetpoint(0);
        controller.setTolerance(0.5, 1);
    }

    @Override
    public void execute() {
        gyroReading = drive.getgyroy();
        speed = controller.calculate(gyroReading);
        // Limit the max power
        if (Math.abs(speed) > 0.6) {
            speed = Math.copySign(0.6, speed);
        }
        // if (speed > 0) {
        //     speed *= 1.1;
        //   }
        drive.arcadeDrive(speed, 0);

        // System.out.println("Current Angle: " + gyroReading);
        // System.out.println("Error " + (0-gyroReading));
        // System.out.println("Drive Power: " + speed);
    }  

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
