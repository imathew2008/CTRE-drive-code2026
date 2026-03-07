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
import edu.wpi.first.math.system.LinearSystem;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class VisionEstimation {
    private static ExtendedKalmanFilter<N3, N3, N3> poseObserver;
    private static CommandSwerveDrivetrain ms;
    private static final double dt = 0.02;
    private static Matrix<N3, N1> lastU = VecBuilder.fill(0.0, 0.0, 0.0);

    private static final Matrix<N3, N1> stateNoise =
            VecBuilder.fill(
                    1e-4,
                    1e-4,
                    1e-4
            );
    private static final Matrix<N3, N1> measurementNoise =
            VecBuilder.fill(
                    1e-4,
                    1e-4,
                    1e-4
            );


    private static final LinearSystem<N3, N3, N3> plant =
            new LinearSystem<>(
                    Matrix.eye(Nat.N3()),
                    Matrix.eye(Nat.N3()).times(dt),
                    Matrix.eye(Nat.N3()),
                    new Matrix<>(Nat.N3(), Nat.N3())
            );
            
    public VisionEstimation(CommandSwerveDrivetrain ms) {
        VisionEstimation.ms = ms;
        poseObserver = new ExtendedKalmanFilter<N3, N3, N3>(
                Nat.N3(),
                Nat.N3(),
                Nat.N3(),
                (x, u) -> u,
                (x, u) -> x,
                VecBuilder.fill(1e-4, 1e-4, 1e-4),
                VecBuilder.fill(1e-4, 1e-4, 1e-4),
                dt
        );
        System.out.println("kalman filter est: " + poseObserver);
        Pose2d realPose = ms.getState().Pose;
    }
    public void update() {
        ChassisSpeeds visionSpeed = ms.getState().Speeds;
        Rotation2d gyroYaw = ms.getPigeon2().getRotation2d();
        ChassisSpeeds fieldSpeeds =
                ChassisSpeeds.fromRobotRelativeSpeeds(visionSpeed, gyroYaw);
        
        Matrix<N3, N1> u =
                VecBuilder.fill(
                        fieldSpeeds.vxMetersPerSecond,
                        fieldSpeeds.vyMetersPerSecond,
                        fieldSpeeds.omegaRadiansPerSecond
                );
        lastU = u;
        poseObserver.predict(u, dt);   
        
    }
    public static void addVisionMeasurement(Pose2d visionPose, double v) {
        Rotation2d gyroYaw = ms.getPigeon2().getRotation2d();
        Matrix<N3, N1> z =
                VecBuilder.fill(
                        visionPose.getX(),
                        visionPose.getY(),
                        gyroYaw.getRadians()
                );

        poseObserver.correct(lastU, z);
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
