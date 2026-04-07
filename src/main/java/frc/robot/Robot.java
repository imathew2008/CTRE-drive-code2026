// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generated.QuestNavConstants;
import frc.robot.subsystems.vision.LimelightHelpers;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import kotlin.Pair;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePiece;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

import static frc.robot.DataClassesKt.zoneName;
import static frc.robot.projectile.BetterSimKt.speedOptimizer;

public class Robot extends TimedRobot {
    QuestNav questNav = new QuestNav();

    private static final String limelight1 = "limelight-two";
    private static final String limelight2 = "limelight-one";
    private static final double megaTag2MaxSpeed = 1.0;

    private Translation2d robotTranslation;
    private Rotation2d robotRotation;
    private ChassisSpeeds robotSpeeds;
    public Vector3 goalPos = new Vector3(4.625594, 4.034536, 1.8288);
    public Vector3 goalPosFloor = new Vector3(4.625594, 4.034536, 0.0);

    private boolean lastshot = false;
    private final Set<GamePieceProjectile> fuelProjectile = new HashSet<>();
    private final StringPublisher currentZone =
            NetworkTableInstance.getDefault()
                    .getStringTopic("CurrentZone")
                    .publish();
    StructArrayPublisher<Pose3d> fuelPublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("MyPoseArray", Pose3d.struct)
            .publish();
    StructPublisher<Pose3d> navXpos = NetworkTableInstance.getDefault().getStructTopic("NavX pos", Pose3d.struct).publish();
//    DoublePublisher posx = NetworkTableInstance.getDefault().getDoubleTopic("pos x").publish();
//    DoublePublisher posy = NetworkTableInstance.getDefault().getDoubleTopic("pos y").publish();
    DoublePublisher rot = NetworkTableInstance.getDefault().getDoubleTopic("rotation").publish();
    DoublePublisher realRot = NetworkTableInstance.getDefault().getDoubleTopic("realRot").publish();
    StructPublisher<Pose2d> posEst = NetworkTableInstance.getDefault().getStructTopic("PosEst", Pose2d.struct)
            .publish();

    public static RobotContainer robotContainer;

    public Robot() {}

    @Override
    public void robotInit() {
        robotTranslation = new Translation2d();
        robotRotation    = new Rotation2d();
        robotSpeeds      = new ChassisSpeeds();
        robotContainer   = new RobotContainer();

        SimulatedArena.getInstance().addGamePiece(new RebuiltFuelOnField(new Translation2d(10, 3)));
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        LimelightHelpers.SetThrottle("limelight1", 100);
        LimelightHelpers.SetThrottle("limelight2", 100);
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        lastshot = robotContainer.shooting();
        LimelightHelpers.SetThrottle("limelight1", 0);
        LimelightHelpers.SetThrottle("limelight2", 0);
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() { CommandScheduler.getInstance().cancelAll(); }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {
        CommandScheduler.getInstance().run();

        ChassisSpeeds speeds = robotContainer.drivetrain.getState().Speeds;
        double speed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        double omegaDegPerSec = Math.toDegrees(speeds.omegaRadiansPerSecond);

        boolean enabled = isEnabled();

        Pose2d estimatedPose = VisionEstimation.getEstimatedPose2d();
        double currentYawDeg = estimatedPose.getRotation().getDegrees();

        if (!enabled) {
            for (String ll : new String[]{limelight1, limelight2}) {
                LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(ll);
                if (mt1.tagCount == 0) continue;

                boolean reject = false;
                if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
                    if (mt1.rawFiducials[0].ambiguity > 0.7)  reject = true;
                    if (mt1.rawFiducials[0].distToCamera > 4) reject = true;
                }
                if (!reject) {
                    VisionEstimation.addLimelightMeasurement(mt1.pose, mt1.timestampSeconds);
                }
            }
        } else if (speed < megaTag2MaxSpeed) {
            for (String ll : new String[]{limelight1, limelight2}) {
                LimelightHelpers.SetRobotOrientation(ll, currentYawDeg, 0, 0, 0, 0, 0);
            }

            LimelightHelpers.PoseEstimate mt2_ll1 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight1);
            LimelightHelpers.PoseEstimate mt2_ll2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight2);

            int combinedTagCount = 0;
            combinedTagCount += mt2_ll1.tagCount;
            combinedTagCount += mt2_ll2.tagCount;

            if (combinedTagCount >= 2 && Math.abs(omegaDegPerSec) < 360) {
                for (LimelightHelpers.PoseEstimate mt2 : new LimelightHelpers.PoseEstimate[]{mt2_ll1, mt2_ll2}) {
                    if (mt2 == null || mt2.tagCount == 0) continue;
                    VisionEstimation.addLimelightMeasurement(mt2.pose, mt2.timestampSeconds);
                }
            }
        }

        questNav.commandPeriodic();

        SmartDashboard.putBoolean("QuestNav/Connected", questNav.isConnected());
        SmartDashboard.putBoolean("QuestNav/Tracking", questNav.isTracking());
        SmartDashboard.putNumber("QuestNav/Latency", questNav.getLatency());


        PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();

        Pose3d robotPoseNavX = null;
        if (poseFrames.length > 0) {

            Pose3d questPose = poseFrames[poseFrames.length - 1].questPose3d();
            robotPoseNavX = questPose.transformBy(QuestNavConstants.ROBOT_TO_QUEST.inverse());

            if(robotPoseNavX != null) {
                VisionEstimation.addQuestMeasurement(
                        robotPoseNavX.toPose2d(),
                        questNav.getLatency()
                );
            }
        }

        Pose2d realPose = robotContainer.drivetrain.getState().Pose;

        int zone = robotContainer.stableZoneLookup.getStableZone(realPose);
        currentZone.set(zoneName(zone));

        robotContainer.visionSim.updateSim(realPose);
        robotContainer.visionEst.update();

//        robotContainer.limelightSource.getVisionUpdate()
//                .ifPresent(vu -> VisionEstimation.addLimelightMeasurement(vu.pose()));

//        navXpos.set(robotContainer.questNavSubsystem.getLatestNoisyQuestNavPose());
        if(navXpos != null) {
            navXpos.set(robotPoseNavX);
        }
//        pos.set(robotContainer.drivetrain.getState().Pose);
        posEst.set(VisionEstimation.getEstimatedPose2d());
//        rot.set(VisionEstimation.getEstimatedPose2d().getRotation().getDegrees());
//        realRot.set(realPose.getRotation().getRadians());

        boolean shoot = robotContainer.shooting();
        if (shoot && !lastshot) {
            robotTranslation = realPose.getTranslation();
            robotRotation = realPose.getRotation();
            robotSpeeds = robotContainer.drivetrain.getState().Speeds;
            Vector3 r0 = new Vector3(realPose.getX(), realPose.getY(), 0.0);
            Vector3 distToGoal = new Vector3(goalPosFloor.minus(r0).getNorm(), goalPos.getZ(), 0.0);
            System.out.println("Distance to goal: " + distToGoal);
            System.out.println("robot: " + r0);
            System.out.println("goal: " + goalPos);

            Pair<Vector3, SimResult> result =
                    speedOptimizer(
                            distToGoal,
                            Vector3.Companion.getZero(),
                            false,
                            false,
                            (i, v, r) -> null
                    );

            LinearVelocity bestSpeed = MetersPerSecond.of(result.getSecond().getResults().get(0).getVel().getNorm());
            Angle bestAngle = Radians.of(HelperFunctionsKt.angle(result.getSecond().getResults().get(0).getVel()));
            LinearVelocity initialSpeed = MetersPerSecond.of(result.getSecond().getResults()
                                    .get(result.getSecond().getResults().size() - 1)
                                    .getVel()
                                    .getNorm());

            Angle initalAngle = Radians.of(HelperFunctionsKt.angle(
                                            result.getSecond().getResults()
                                            .get(result.getSecond().getResults().size() - 1).getVel()));

            System.out.println("Best Speed: " + bestSpeed);
            System.out.println("Best Angle: " + bestAngle.in(Degrees));
            System.out.println("Final Angle: " + initalAngle.in(Degrees));
            System.out.println("Initial Speed: " + initialSpeed);

            RebuiltFuelOnFly fuelOnFly = new RebuiltFuelOnFly(
                    robotTranslation,
                    new Translation2d(0, 0),
                    robotSpeeds,
                    robotRotation,
                    Meters.of(0.5461),
                    bestSpeed,
                    bestAngle
            );
            fuelOnFly.launch();
            SimulatedArena.getInstance().addGamePieceProjectile(fuelOnFly);
            System.out.println("projectiles=" + fuelProjectile.size());
        }

        lastshot = shoot;
        SimulatedArena.getInstance().simulationPeriodic();
        GamePieceProjectile.updateGamePieceProjectiles(SimulatedArena.getInstance(), fuelProjectile);

        ((StructArrayPublisher<Object>) (Object) fuelPublisher).accept(
                SimulatedArena.getInstance()
                        .getGamePiecesByType("RebuiltFuelOnField")
                        .stream()
                        .map(GamePiece::getPose3d)
                        .toArray()
        );
    }
}