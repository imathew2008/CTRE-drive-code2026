package frc.robot.subsystems.questNav;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Vector2;

public class QuestNavSimModel {
    private final GyroPointKinematic posErrorSim;

    public QuestNavSimModel(double robotSpeed) {
        posErrorSim = new GyroPointKinematic(robotSpeed);
    }

    public Pose2d applyError(Pose2d truePose, double robotSpeedMetersPerSec, double dt) {
        posErrorSim.update(robotSpeedMetersPerSec, dt);

        Vector2 err = posErrorSim.position();

        return new Pose2d(
                truePose.getX() + err.getX(),
                truePose.getY() + err.getY(),
                truePose.getRotation()
        );
    }

    public Pose2d previewErrorPose(Pose2d truePose) {
        Vector2 err = posErrorSim.position();

        return new Pose2d(
                truePose.getX() + err.getX(),
                truePose.getY() + err.getY(),
                truePose.getRotation()
        );
    }

    public Vector2 getErrorVector() {
        return posErrorSim.position();
    }
}