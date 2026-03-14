package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionSim {
    private final PhotonCamera cameraOne;
    private final PhotonCamera cameraTwo;
    private final VisionSystemSim visionSim;

    public VisionSim(String cameraNameOne, String cameraNameTwo, Transform3d robotToCameraOne, Transform3d robotToCameraTwo, AprilTagFieldLayout tags) {
        cameraOne = new PhotonCamera(cameraNameOne);
        cameraTwo = new  PhotonCamera(cameraNameTwo);
        visionSim = new VisionSystemSim("photonvision");
        visionSim.addAprilTags(tags);
        SimCameraProperties cameraProperties = new SimCameraProperties();

        PhotonCameraSim cameraSimOne = new PhotonCameraSim(cameraOne, cameraProperties);
        PhotonCameraSim cameraSimTwo = new PhotonCameraSim(cameraTwo, cameraProperties);
        visionSim.addCamera(cameraSimOne, robotToCameraOne);
        visionSim.addCamera(cameraSimTwo, robotToCameraTwo);
    }
    
    public void updateSim(Pose2d robotPose) {
        visionSim.update(robotPose);
    }
    public PhotonCamera getCameraOne() {
        return cameraOne;
    }
    public PhotonCamera getCameraTwo() {
        return cameraTwo;
    }
}