package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase{
    
    PhotonCamera camera = new PhotonCamera("photonvision");

    public Vision() {}

    public PhotonPipelineResult getCamera1Results() {
        // get results for the camera
        return camera.getLatestResult();
    }

    public void enableCamera1VisionMode() {
        // enable vision processer mode
        camera.setDriverMode(false);
    }

    public void enableCamera1DriveMode() {
        // enable driver camera mode
        camera.setDriverMode(true);
    }

    @Override
    public void periodic() {}
}
