package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.ArmMotorConstants;
import frc.robot.subsystems.Arm;

public class ArmCommand extends PIDCommand{
    Arm arm;

    private int location;

    public ArmCommand(Arm arm, double speed, int location) {
        super(new PIDController(ArmMotorConstants.kp, ArmMotorConstants.ki, ArmMotorConstants.kd), 
        arm::GetEncoderRotation, location, ouput -> arm.Run(speed), arm);
        this.arm = arm;
        this.location = location;
        getController().enableContinuousInput(-180, 180);
        // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
        // setpoint before it is considered as having reached the reference
        getController()
            .setTolerance(ArmMotorConstants.kArmToleranceMeter, ArmMotorConstants.kArmRateToleranceMeterPerS);
    }

    @Override
    public void initialize() {
        arm.resetEncoders();
    }

    public void setLocation(int location) {
        this.location = location;
        
    }

    @Override
    public boolean isFinished() {
      // End when the controller is at the reference.
      return getController().atSetpoint();
    }
}
