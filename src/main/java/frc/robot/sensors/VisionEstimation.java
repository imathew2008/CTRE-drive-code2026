package frc.robot.sensors;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

public class VisionEstimation {
    private static CommandSwerveDrivetrain drivetrain;
    private static SwerveDrivePoseEstimator poseEstimator;

    public VisionEstimation(CommandSwerveDrivetrain drivetrain) {
        VisionEstimation.drivetrain = drivetrain;

        Matrix<N3, N1> stateStdDevs = VecBuilder.fill(
                0.05, // x meters
                0.05, // y meters
                0.03  // theta radians
        );

        Matrix<N3, N1> visionStdDevs = VecBuilder.fill(
                0.15, // x meters
                0.15, // y meters
                1e4  // theta radians
        );

        poseEstimator = new SwerveDrivePoseEstimator(
                getKinematics(),
                getGyroYaw(),
                getModulePositions(),
                new Pose2d(),
                stateStdDevs,
                visionStdDevs
        );
    }

    public void update() {
        if (poseEstimator == null || drivetrain == null) {
            return;
        }

        poseEstimator.update(
                getGyroYaw(),
                getModulePositions()
        );
    }

    public static void addVisionMeasurement(
            Pose2d measuredPose,
            double timestampSeconds,
            Matrix<N3, N1> visionStdDevs
    ) {
        if (poseEstimator == null) {
            return;
        }

        poseEstimator.addVisionMeasurement(
                measuredPose,
                timestampSeconds,
                visionStdDevs
        );
    }

    public static void addVisionMeasurement(
            Pose2d measuredPose,
            double timestampSeconds
    ) {
        if (poseEstimator == null) {
            return;
        }

        poseEstimator.addVisionMeasurement(
                measuredPose,
                timestampSeconds
        );
    }

    public static void addQuestMeasurement(
            Pose2d measuredPose,
            double latencySeconds,
            Matrix<N3, N1> visionStdDevs
    ) {
        if (poseEstimator == null) {
            return;
        }

        poseEstimator.addVisionMeasurement(
                measuredPose,
                latencySeconds,
                visionStdDevs
        );
    }

    public static void addQuestMeasurement(
            Pose2d measuredPose,
            double latencySeconds
    ) {
        addQuestMeasurement(
                measuredPose,
                latencySeconds,
                VecBuilder.fill(
                        0.01, // x meters
                        0.01, // y meters
                        1e-4  // theta radians
                )
        );
    }

    public static void addLimelightMeasurement(
            Pose2d measuredPose,
            double latencySeconds,
            Matrix<N3, N1> visionStdDevs
    ) {
        if (poseEstimator == null) {
            return;
        }

        poseEstimator.addVisionMeasurement(
                measuredPose,
                latencySeconds,
                visionStdDevs
        );
    }

    public static void addLimelightMeasurement(
            Pose2d measuredPose,
            double latencySeconds
    ) {
        double timestampSeconds = Timer.getFPGATimestamp() - latencySeconds;

        addLimelightMeasurement(
                measuredPose,
                timestampSeconds,
                VecBuilder.fill(
                        0.20, // x meters
                        0.20, // y meters
                        1e4   // theta radians
                )
        );
    }

    public static Pose2d getEstimatedPose2d() {
        if (poseEstimator == null) {
            return new Pose2d();
        }
        return poseEstimator.getEstimatedPosition();
    }

    public static void resetPose(Pose2d pose) {
        if (poseEstimator == null || drivetrain == null) {
            return;
        }

        poseEstimator.resetPosition(
                getGyroYaw(),
                getModulePositions(),
                pose
        );
    }

    public static void setVisionMeasurementStdDevs(Matrix<N3, N1> visionStdDevs) {
        if (poseEstimator == null) {
            return;
        }

        poseEstimator.setVisionMeasurementStdDevs(visionStdDevs);
    }

    private static Rotation2d getGyroYaw() {
        return drivetrain.getPigeon2().getRotation2d();
    }

    private static SwerveModulePosition[] getModulePositions() {
        return drivetrain.getState().ModulePositions;
    }

    private static SwerveDriveKinematics getKinematics() {
        return drivetrain.getKinematics();
    }
}