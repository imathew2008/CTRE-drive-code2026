package frc.robot.generated;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public final class QuestNavConstants {

    // Robot → Quest transform (meters, radians)
    public static final Transform3d ROBOT_TO_QUEST =
            new Transform3d(
                    new Translation3d(
                            0.15,   // X forward (meters)  (replace with real values)
                            0.00,   // Y left
                            0.25    // Z up
                    ),
                    new Rotation3d(
                            0.0,                    // roll
                            0.0,                    // pitch
                            Math.toRadians(180.0)   // yaw
                    )
            );

    private QuestNavConstants() {}
}