package frc.robot.sensors;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.QuestNavConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;


public class QuestNavSubsystem extends SubsystemBase {
    private final QuestNav questNav;

    private static final Matrix<N3, N3> QUESTNAV_R = new Matrix<>(Nat.N3(), Nat.N3());

    static {
        QUESTNAV_R.set(0, 0, 0.05 * 0.05);
        QUESTNAV_R.set(1, 1, 0.05 * 0.05);
        QUESTNAV_R.set(2, 2, 0.07 * 0.07);
    }

    public QuestNavSubsystem() {
        questNav = new QuestNav();
    }

    public Matrix<N3, N3> getQuestNavR() {
        return QUESTNAV_R;
    }

    @Override
    public void periodic() {
        questNav.commandPeriodic();
        SmartDashboard.putBoolean("QuestNav/Connected", questNav.isConnected());
        SmartDashboard.putBoolean("QuestNav/Tracking", questNav.isTracking());
        SmartDashboard.putNumber("QuestNav/Latency", questNav.getLatency());

        PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();
        if (poseFrames.length > 0) {
            Pose3d questPose = poseFrames[poseFrames.length - 1].questPose3d();
            Pose3d robotPose = questPose.transformBy(QuestNavConstants.ROBOT_TO_QUEST.inverse());
            VisionEstimation.addQuestMeasurement(robotPose.toPose2d(), questNav.getLatency() * 1000.0);
        }
    }
}