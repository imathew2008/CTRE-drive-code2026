// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashSet;
import java.util.Set;

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
    StructPublisher<Pose2d> navXpos = NetworkTableInstance.getDefault().getStructTopic("NavX pos", Pose2d.struct).publish();
    DoublePublisher posx = NetworkTableInstance.getDefault().getDoubleTopic("pos x").publish();
    DoublePublisher posy = NetworkTableInstance.getDefault().getDoubleTopic("pos y").publish();
    DoublePublisher rot = NetworkTableInstance.getDefault().getDoubleTopic("rotation").publish();
    DoublePublisher realRot = NetworkTableInstance.getDefault().getDoubleTopic("realRot").publish();
    StructPublisher<Pose2d> pos = NetworkTableInstance.getDefault().getStructTopic("Real Pos", Pose2d.struct)
            .publish();

    public static RobotContainer robotContainer;

    public Robot() {

    }

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
    public void disabledInit() {}

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
        Pose2d realPose = robotContainer.drivetrain.getState().Pose;

        int zone = robotContainer.stableZoneLookup.getStableZone(realPose);
        currentZone.set(zoneName(zone));

        robotContainer.visionSim.updateSim(realPose);
        robotContainer.visionEst.update();

        robotContainer.limelightSource.getVisionUpdate()
                .ifPresent(vu -> VisionEstimation.addPoseMeasurement(vu.pose()));

        navXpos.set(robotContainer.questNavSubsystem.getLatestNoisyQuestNavPose());
        pos.set(robotContainer.drivetrain.getState().Pose);
        posx.set(VisionEstimation.getEstimatedPose2d().getTranslation().getX());
        posy.set(VisionEstimation.getEstimatedPose2d().getTranslation().getY());
        rot.set(VisionEstimation.getEstimatedPose2d().getRotation().getRadians());
        realRot.set(realPose.getRotation().getRadians());

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