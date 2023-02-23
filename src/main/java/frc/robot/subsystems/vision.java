package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class vision extends SubsystemBase{
    
    PhotonCamera camera1 = new PhotonCamera("photonvision");
    NetworkTable camera2 = NetworkTableInstance.getDefault().getTable("limelight");

    public vision() {
        CameraServer.addAxisCamera("photonvision.local:1181");
    }

    public PhotonPipelineResult getCamera1Results() {
        return camera1.getLatestResult();
    }

    public void enableCamera1VisionMode() {
        camera1.setDriverMode(false);
    }

    public void enableCamera1DriveMode() {
        camera1.setDriverMode(true);
    }

    public NetworkTableEntry getCamera2Results() {
        return camera2.getEntry("targetpose_cameraspace");
    }

    public void enableCamera2DriveMode() {
        camera2.getEntry("camMode").setNumber(1);
    }

    public void enableCamera2VisionMode() {
        camera2.getEntry("camMode").setNumber(0);
    }

    @Override
    public void periodic() {}
}
