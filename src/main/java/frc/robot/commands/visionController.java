package frc.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.endAffectorConstants;
import frc.robot.subsystems.endAffector;
import frc.robot.subsystems.vision;

public class visionController extends CommandBase{
    
    // var setup
    private XboxController m_Controller;
    private vision m_visionSubsystem;
    private endAffector m_claw;

    public visionController(XboxController controller, vision visionSubsystem, endAffector claw) {
        // typical stuff
        m_Controller = controller;
        m_visionSubsystem = visionSubsystem;
        m_claw = claw;
        addRequirements(visionSubsystem, m_claw);
    }

    @Override
    public void execute() {
        // should I run vision processing?
        if (m_Controller.getRawButton(IOConstants.backButtonChannel)) {
            var result = m_visionSubsystem.getCamera1Results();
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
                if (0.247775 - Units.feetToMeters(m_claw.getCenterPos()*endAffectorConstants.tick2Feet)  > neededDistance) {
                    m_claw.runBoth(0.3);
                }
                else if (0.247775 - Units.feetToMeters(m_claw.getCenterPos()*endAffectorConstants.tick2Feet)  > neededDistance) {
                    m_claw.runBoth(-0.3);
                }
            }
        }
    }
}
