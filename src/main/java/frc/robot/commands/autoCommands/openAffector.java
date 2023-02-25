package frc.robot.commands.autoCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.endAffectorConstants;
import frc.robot.subsystems.endAffector;

public class openAffector extends PIDCommand{


  endAffector affector;
  double distance;

  public openAffector(endAffector affector, double distance) {
    // setup the pid command
    super(new PIDController(endAffectorConstants.kp, endAffectorConstants.ki, endAffectorConstants.kd), 
    affector::getLeftEncoder, distance, output -> affector.runLeft(output), affector);
    // typical stuff
    this.affector = affector;
    getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController().setTolerance(endAffectorConstants.kAffectorToleranceMeter,
      endAffectorConstants.kAffectorRateToleranceMeterPerS);
  }

  @Override
  public boolean isFinished() {
    // am I finished?
    return getController().atSetpoint();
  }
}
