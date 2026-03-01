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
    private final PhotonCamera cameraOne;
    private final PhotonCamera cameraTwo;
    private final PhotonPoseEstimator poseEstimatorOne;
    private final PhotonPoseEstimator poseEstimatorTwo;
    
    public record VisionUpdate(Pose2d pose, double timestampSeconds) {}
    
    public LimelightMeasurementSource(PhotonCamera cameraOne, PhotonCamera cameraTwo, AprilTagFieldLayout tags, Transform3d robotToCameraOne, Transform3d robotToCameraTwo)
    {
        this.cameraOne = cameraOne;
        this.cameraTwo = cameraTwo;
        this.poseEstimatorOne = new PhotonPoseEstimator(
                tags,
                robotToCameraOne
        );
        this.poseEstimatorTwo = new PhotonPoseEstimator(
                tags,
                robotToCameraTwo
        );
        
    }

    public List<VisionUpdate> getVisionUpdate()
    {
        List<VisionUpdate> updates = new java.util.ArrayList<>();

        List<PhotonPipelineResult> resultOne = cameraOne.getAllUnreadResults();
        List<PhotonPipelineResult> resultTwo = cameraTwo.getAllUnreadResults();

        if (!resultOne.isEmpty()) {
            var estOne = poseEstimatorOne.estimateLowestAmbiguityPose(
                    resultOne.get(resultOne.size() - 1)
            );

            if (estOne.isPresent()) {
                Pose2d pose = estOne.get().estimatedPose.toPose2d();
                double ts = estOne.get().timestampSeconds;
                updates.add(new VisionUpdate(pose, ts));
            }
        }

        if (!resultTwo.isEmpty()) {
            var estTwo = poseEstimatorTwo.estimateLowestAmbiguityPose(
                    resultTwo.get(resultTwo.size() - 1)
            );

            if (estTwo.isPresent()) {
                Pose2d pose = estTwo.get().estimatedPose.toPose2d();
                double ts = estTwo.get().timestampSeconds;
                updates.add(new VisionUpdate(pose, ts));
            }
        }

        return updates;
    }
    
}
