package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionSim {
    private final PhotonCamera camera;
    private final VisionSystemSim visionSim;
    private final PhotonCameraSim cameraSim;
    
    public VisionSim(String cameraName, Transform3d robotToCamera, AprilTagFieldLayout tags) {
        camera = new PhotonCamera(cameraName);
        visionSim = new VisionSystemSim("photonvision");
        visionSim.addAprilTags(tags);
        SimCameraProperties cameraProperties = new SimCameraProperties();
        
        cameraSim = new PhotonCameraSim(camera, cameraProperties);
        visionSim.addCamera(cameraSim, robotToCamera);
    }
    
    public void updateSim(Pose2d robotPose) {
        visionSim.update(robotPose);
    }
    public PhotonCamera getCamera() {
        return camera;
    }
}