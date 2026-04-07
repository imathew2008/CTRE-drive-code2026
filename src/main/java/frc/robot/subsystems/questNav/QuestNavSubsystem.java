package frc.robot.subsystems.questNav;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.VisionEstimation;
import frc.robot.generated.QuestNavConstants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class QuestNavSubsystem extends SubsystemBase {

    private final QuestNav questNav = new QuestNav();
    private final CommandSwerveDrivetrain drivetrain;
    private final QuestNavSimModel questNavSim;

    private Pose2d latestNoisyQuestNavPose = new Pose2d();

    private static final Matrix<N3, N3> QUESTNAV_R = new Matrix<>(Nat.N3(), Nat.N3());

    static {
        QUESTNAV_R.set(0, 0, 0.05 * 0.05);
        QUESTNAV_R.set(1, 1, 0.05 * 0.05);
        QUESTNAV_R.set(2, 2, 0.07 * 0.07);
    }

    public QuestNavSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.questNavSim = new QuestNavSimModel(1.0);
    }

    public Matrix<N3, N3> getQuestNavR() {
        return QUESTNAV_R;
    }

    public Pose2d getLatestNoisyQuestNavPose() {
        return latestNoisyQuestNavPose;
    }


    @Override
    public void periodic() {
        if (RobotBase.isSimulation()) {
            Pose2d truePose = drivetrain.getState().Pose;
            ChassisSpeeds speeds = drivetrain.getState().Speeds;
            double robotSpeed =
                    Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

            Pose2d noisyQuestNavPose =
                    questNavSim.applyError(truePose, robotSpeed, 0.02);

            latestNoisyQuestNavPose = noisyQuestNavPose;

//            VisionEstimation.addPoseMeasurement(noisyQuestNavPose, getQuestNavR());
            return;
        }

//        SmartDashboard.putBoolean("QuestNav/Connected", questNav.isConnected());
//        SmartDashboard.putBoolean("QuestNav/Tracking", questNav.isTracking());
//        SmartDashboard.putNumber("QuestNav/Latency", questNav.getLatency());
//
//
//        PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();
//
//        Pose3d robotPose = null;
//        if (poseFrames.length > 0) {
//
//            Pose3d questPose = poseFrames[poseFrames.length - 1].questPose3d();
//            robotPose = questPose.transformBy(QuestNavConstants.ROBOT_TO_QUEST.inverse());
//        }
//
//        latestNoisyQuestNavPose = robotPose.toPose2d();
//
//        VisionEstimation.addPoseMeasurement(robotPose.toPose2d(), getQuestNavR());
    }
}