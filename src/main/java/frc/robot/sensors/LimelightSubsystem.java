package frc.robot.sensors;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import gg.questnav.questnav.QuestNav;

import static edu.wpi.first.wpilibj.RobotState.isEnabled;

public class LimelightSubsystem extends SubsystemBase {
    private static final String limelight1 = "limelight-two";
    private static final String limelight2 = "limelight-one";

    private final CommandSwerveDrivetrain drivetrain;
    private final QuestNav quest;

    public LimelightSubsystem(CommandSwerveDrivetrain drivetrain, QuestNav quest) {
        this.drivetrain = drivetrain;
        this.quest = quest;
    }

    public Pose2d getPos() {
        return LimelightHelpers.getBotPose2d("limelight-one");
    }

    @Override
    public void periodic() {
        boolean doRejectUpdate = false;

//        if (false) {
//            LimelightHelpers.SetThrottle("limelight-front", 100);
//            LimelightHelpers.SetThrottle("limelight-back", 100);
//
//            for (String ll : new String[]{limelight1, limelight2}) {
//                LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(ll);
//
//                if (mt1.tagCount > 0 && mt1.rawFiducials.length > 0) {
//                    if (mt1.rawFiducials[0].ambiguity > .7) {
//                        doRejectUpdate = true;
//                    }
//                    if (mt1.rawFiducials[0].distToCamera > 3) {
//                        doRejectUpdate = true;
//                    }
//                }
//                if (mt1.tagCount == 0) {
//                    doRejectUpdate = true;
//                }
//
//                if (!doRejectUpdate) {
//                    VisionEstimation.addLimelightMeasurement(
//                            mt1.pose,
//                            mt1.timestampSeconds);
//                }
//            }

        if(true) {
            LimelightHelpers.SetThrottle("limelight-front", 0);
            LimelightHelpers.SetThrottle("limelight-back", 0);

            LimelightHelpers.SetRobotOrientation("limelight-front", drivetrain.getPigeon2()
                    .getRotation2d().getDegrees(), 0, 0, 0, 0, 0);
            LimelightHelpers.SetRobotOrientation("limelight-back", drivetrain.getPigeon2()
                    .getRotation2d().getDegrees(), 0, 0, 0, 0, 0);

            LimelightHelpers.PoseEstimate ll1 =
                    LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-front");
            LimelightHelpers.PoseEstimate ll2 =
                    LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-back");

            if (Math.abs(drivetrain.getPigeon2().getAngularVelocityXDevice().getValueAsDouble()) > 360) {
                doRejectUpdate = true;
            }
            if (ll2.tagCount + ll1.tagCount < 1) {
                doRejectUpdate = true;
            }
            if (!doRejectUpdate) {
                if (ll1.tagCount > 0) {
                    VisionEstimation.addLimelightMeasurement(
                            ll1.pose,
                            ll1.timestampSeconds);
                    quest.setPose(new Pose3d(ll1.pose.getX(), ll1.pose.getY(), 0.0,
                            new Rotation3d(0.0, 0.0, 0.0)));
                }

                if (ll2.tagCount > 0) {
                    VisionEstimation.addLimelightMeasurement(
                            ll2.pose,
                            ll2.timestampSeconds);

                    quest.setPose(new Pose3d(ll2.pose.getX(), ll2.pose.getY(), 0.0,
                            new Rotation3d(0.0, 0.0, 0.0)));
                }
            }
        }
        VisionEstimation.update();
        SmartDashboard.putString("aaaaaaaPos", VisionEstimation.getEstimatedPose2d().toString());
    }
}