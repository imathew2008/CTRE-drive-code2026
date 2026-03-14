package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.ExtendedKalmanFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class VisionEstimation {
    private static ExtendedKalmanFilter<N3, N3, N3> poseObserver;
    private static CommandSwerveDrivetrain ms;
    private static final double dt = 0.02;
    private static Matrix<N3, N1> lastU = VecBuilder.fill(0.0, 0.0, 0.0);

    public VisionEstimation(CommandSwerveDrivetrain ms) {
        VisionEstimation.ms = ms;
        poseObserver = new ExtendedKalmanFilter<>(
                Nat.N3(),
                Nat.N3(),
                Nat.N3(),
                (x, u) -> u,
                (x, u) -> x,
                VecBuilder.fill(1e-4, 1e-4, 1e-4),
                VecBuilder.fill(1e-4, 1e-4, 1e-4),
                dt
        );
    }

    public void update() {
        ChassisSpeeds robotRelativeSpeeds = ms.getState().Speeds;
        Rotation2d gyroYaw = ms.getPigeon2().getRotation2d();

        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, gyroYaw);

        lastU = VecBuilder.fill(
                fieldSpeeds.vxMetersPerSecond,
                fieldSpeeds.vyMetersPerSecond,
                fieldSpeeds.omegaRadiansPerSecond
        );

        poseObserver.predict(lastU, dt);
    }

    public static void addPoseMeasurement(Pose2d measuredPose) {
        Rotation2d gyroYaw = ms.getPigeon2().getRotation2d();

        Matrix<N3, N1> z = VecBuilder.fill(
                measuredPose.getX(),
                measuredPose.getY(),
                gyroYaw.getRadians()
        );

        poseObserver.correct(lastU, z);
    }

    public static void addPoseMeasurement(
            Pose2d measuredPose,
            Matrix<N3, N3> measurementCovariance
    ) {
        Rotation2d gyroYaw = ms.getPigeon2().getRotation2d();

        Matrix<N3, N1> z = VecBuilder.fill(
                measuredPose.getX(),
                measuredPose.getY(),
                gyroYaw.getRadians()
        );

        poseObserver.correct(
                Nat.N3(),
                lastU,
                z,
                (x, u) -> x,
                measurementCovariance
        );
    }

    public static Pose2d getEstimatedPose2d() {
        Matrix<N3, N1> x = poseObserver.getXhat();
        return new Pose2d(
                x.get(0, 0),
                x.get(1, 0),
                new Rotation2d(x.get(2, 0))
        );
    }
}