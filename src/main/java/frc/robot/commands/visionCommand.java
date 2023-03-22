package frc.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.endEffectorConstants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.vision;

public class visionCommand extends CommandBase{
    
    // var setup
    private final XboxController controller;
    private final vision visionSubsystem;
    private final Claw claw;

    public visionCommand(XboxController controller, vision visionSubsystem, Claw claw) {
        // typical stuff
        this.controller = controller;
        this.visionSubsystem = visionSubsystem;
        this.claw = claw;
        addRequirements(visionSubsystem, this.claw);
    }

    @Override
    public void execute() {
        // should I run vision processing?
        if (controller.getRawButton(IOConstants.backButtonChannel)) {
            var result = visionSubsystem.getCamera1Results();
            // do I have targets?
            if (result.hasTargets()) {
                // set up the target
                PhotonTrackedTarget target = result.getBestTarget();
                Transform3d pos = target.getBestCameraToTarget();
                double neededDistance;

                // set up the distance
                if (pos.getY() < 0) {
                    neededDistance = Units.feetToMeters(-18.5) - pos.getY();
                }
                else {
                    neededDistance = Units.feetToMeters(18.5) - pos.getY();
                }

                // run the motors in the correct dir
                if (0.247775 - Units.feetToMeters(claw.getCenterPos()* endEffectorConstants.tick2Feet)  > neededDistance) {
                    claw.runBoth(0.3);
                }
                else if (0.247775 - Units.feetToMeters(claw.getCenterPos()* endEffectorConstants.tick2Feet)  > neededDistance) {
                    claw.runBoth(-0.3);
                }
            }
        }
    }
}
