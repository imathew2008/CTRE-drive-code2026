package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import org.jetbrains.annotations.NotNull;
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
    
    public LimelightMeasurementSource(PhotonCamera cameraOne, PhotonCamera cameraTwo, AprilTagFieldLayout tags,
                                      Transform3d robotToCameraOne, Transform3d robotToCameraTwo) {
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

    public Optional<VisionUpdate> getVisionUpdate()
    {
        VisionUpdate v1 = null;
        VisionUpdate v2 = null;
        
        List<PhotonPipelineResult> resultsOne = cameraOne.getAllUnreadResults();
        if (!resultsOne.isEmpty()) {
            PhotonPipelineResult latest = resultsOne.get(resultsOne.size() - 1);
            var estOne = poseEstimatorOne.estimateCoprocMultiTagPose(latest);

            if (estOne.isPresent()) {
                Pose2d pose = estOne.get().estimatedPose.toPose2d();
                double ts = estOne.get().timestampSeconds;
                v1 = new VisionUpdate(pose, ts);
            }
        }
        
        List<PhotonPipelineResult> resultsTwo = cameraTwo.getAllUnreadResults();
        if (!resultsTwo.isEmpty()) {
            PhotonPipelineResult latest = resultsTwo.get(resultsTwo.size() - 1);
            var estTwo = poseEstimatorTwo.estimateCoprocMultiTagPose(latest);

            if (estTwo.isPresent()) {
                Pose2d pose = estTwo.get().estimatedPose.toPose2d();
                double ts = estTwo.get().timestampSeconds;
                v2 = new VisionUpdate(pose, ts);
            }
        }
        

        if (v1 == null && v2 == null)
            return Optional.empty();

        if (v1 != null && v2 == null)
            return Optional.of(v1);

        if (v1 == null)
            return Optional.of(v2);

        Pose2d fusedPose = getPose2d(v1, v2);

        double fusedTimestamp =
                Math.max(v1.timestampSeconds(), v2.timestampSeconds());

        return Optional.of(new VisionUpdate(fusedPose, fusedTimestamp));
    }

    @NotNull
    private static Pose2d getPose2d(VisionUpdate v1, VisionUpdate v2) {
        Pose2d p1 = v1.pose();
        Pose2d p2 = v2.pose();

        double w1 = 0.5;
        double w2 = 0.5;

        double x = w1 * p1.getX() + w2 * p2.getX();
        double y = w1 * p1.getY() + w2 * p2.getY();

        double cos =
                w1 * Math.cos(p1.getRotation().getRadians()) +
                        w2 * Math.cos(p2.getRotation().getRadians());

        double sin =
                w1 * Math.sin(p1.getRotation().getRadians()) +
                        w2 * Math.sin(p2.getRotation().getRadians());

        double theta = Math.atan2(sin, cos);

        return new Pose2d(x, y, new Rotation2d(theta));
    }

}
