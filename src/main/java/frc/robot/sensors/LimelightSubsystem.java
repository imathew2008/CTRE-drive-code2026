package frc.robot.sensors;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

import static edu.wpi.first.wpilibj.RobotState.isEnabled;

public class LimelightSubsystem extends SubsystemBase {
    private static final String limelight1 = "limelight-two";
    private static final String limelight2 = "limelight-one";

    private final CommandSwerveDrivetrain drivetrain;

    public LimelightSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void periodic() {
        boolean doRejectUpdate = false;

        if (!isEnabled()) {
            LimelightHelpers.SetThrottle("limelight-one", 100);
            LimelightHelpers.SetThrottle("limelight-two", 100);

            for (String ll : new String[]{limelight1, limelight2}) {
                LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(ll);

                if (mt1.tagCount > 0 && mt1.rawFiducials.length > 0) {
                    if (mt1.rawFiducials[0].ambiguity > .7) {
                        doRejectUpdate = true;
                    }
                    if (mt1.rawFiducials[0].distToCamera > 3) {
                        doRejectUpdate = true;
                    }
                }
                if (mt1.tagCount == 0) {
                    doRejectUpdate = true;
                }

                if (!doRejectUpdate) {
                    VisionEstimation.addLimelightMeasurement(
                            mt1.pose,
                            mt1.timestampSeconds);
                }
            }

        } else {
            LimelightHelpers.SetThrottle("limelight-one", 0);
            LimelightHelpers.SetThrottle("limelight-two", 0);

            LimelightHelpers.SetRobotOrientation("limelight-one", drivetrain.getPigeon2()
                    .getRotation2d().getDegrees(), 0, 0, 0, 0, 0);
            LimelightHelpers.SetRobotOrientation("limelight-two", drivetrain.getPigeon2()
                    .getRotation2d().getDegrees(), 0, 0, 0, 0, 0);

            LimelightHelpers.PoseEstimate ll1 =
                    LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-one");
            LimelightHelpers.PoseEstimate ll2 =
                    LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-two");

            if (Math.abs(drivetrain.getPigeon2().getAngularVelocityXDevice().getValueAsDouble()) > 360) {
                doRejectUpdate = true;
            }
            if (ll2.tagCount + ll1.tagCount < 2) {
                doRejectUpdate = true;
            }
            if (!doRejectUpdate) {
                if (ll1.tagCount > 0) {
                    VisionEstimation.addLimelightMeasurement(
                            ll1.pose,
                            ll1.timestampSeconds);
                }

                if (ll2.tagCount > 0) {
                    VisionEstimation.addLimelightMeasurement(
                            ll2.pose,
                            ll2.timestampSeconds);
                }
            }
        }
    }
}