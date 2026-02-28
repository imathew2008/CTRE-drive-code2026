package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.List;
import java.util.Optional;

public class LimelightMeasurementSource
{
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;
    
    public record VisionUpdate(Pose2d pose, double timestampSeconds) {}
    
    public LimelightMeasurementSource(PhotonCamera camera, AprilTagFieldLayout tags, Transform3d robotToCamera)
    {
        this.camera = camera;
        
        this.poseEstimator = new PhotonPoseEstimator(
                tags,
                robotToCamera
        );
    }
    
    public Optional<VisionUpdate> getVisionUpdate()
    {
        List<PhotonPipelineResult> result = camera.getAllUnreadResults();
        if(result.isEmpty()) return Optional.empty();

        Optional<EstimatedRobotPose> est = poseEstimator.estimateLowestAmbiguityPose(result.get(result.size() - 1));
        if(est.isEmpty()) return Optional.empty();
        
        Pose2d pose2d = est.get().estimatedPose.toPose2d();
        double ts = est.get().timestampSeconds;
        
        return Optional.of(new VisionUpdate(pose2d, ts));
    }
    
}
