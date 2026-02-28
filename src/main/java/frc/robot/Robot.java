// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePiece;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

import java.util.HashSet;
import java.util.Set;

import static edu.wpi.first.units.Units.*;

public class Robot extends TimedRobot {
    private Translation2d robotTranslation;
    private Rotation2d robotRotation;
    private ChassisSpeeds robotSpeeds;

    private RebuiltFuelOnFly fuelOnFly;
    private boolean lastshot = false;
    private final Set<GamePieceProjectile> fuelProjectile = new HashSet<>();
    StructArrayPublisher<Pose3d> fuelPublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("MyPoseArray", Pose3d.struct)
            .publish();

    StructPublisher<Pose2d> pos = NetworkTableInstance.getDefault().getStructTopic("PredictedPos", Pose2d.struct)
            .publish();
    DoublePublisher gyroPos = NetworkTableInstance.getDefault().getDoubleTopic("Pigeon2 Pos").publish();
    DoublePublisher posx = NetworkTableInstance.getDefault().getDoubleTopic("pos x").publish();
    DoublePublisher posy = NetworkTableInstance.getDefault().getDoubleTopic("pos y").publish();
    DoublePublisher rot = NetworkTableInstance.getDefault().getDoubleTopic("rotation").publish();
    DoublePublisher realRot = NetworkTableInstance.getDefault().getDoubleTopic("realRot").publish();


    public static RobotContainer robotContainer;

    public Robot() {

    }

    @Override
    public void robotInit() {
        robotTranslation = new Translation2d();
        robotRotation = new Rotation2d();
        robotSpeeds = new ChassisSpeeds();
        robotContainer = new RobotContainer();

        SimulatedArena.getInstance().addGamePiece(new RebuiltFuelOnField(
                new Translation2d(10, 3)));

        NetworkTable table =
                NetworkTableInstance.getDefault().getTable("FieldSimulation");

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
    public void testInit() {CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {
        Pose2d realPose = robotContainer.drivetrain.getState().Pose;

        robotContainer.visionSim.updateSim(realPose);
        robotContainer.visionEst.update();

        robotContainer.limelightSource.getVisionUpdate().ifPresent(vu -> {
            VisionEstimation.addVisionMeasurement(vu.pose(), vu.timestampSeconds());
        });

        pos.set(VisionEstimation.getEstimatedPose2d());
        posx.set(VisionEstimation.getEstimatedPose2d().getTranslation().getX());
        posy.set(VisionEstimation.getEstimatedPose2d().getTranslation().getY());
        rot.set(VisionEstimation.getEstimatedPose2d().getRotation().getRadians());
        realRot.set(realPose.getRotation().getRadians());

        boolean shoot = robotContainer.shooting();
        if(shoot && !lastshot)
        {
            robotTranslation = realPose.getTranslation();
            robotRotation = realPose.getRotation();
            robotSpeeds = robotContainer.drivetrain.getState().Speeds;
            fuelOnFly = new RebuiltFuelOnFly(
                    robotTranslation,                 // live robot position
                    new Translation2d(0, 0),      // shooter offset
                    robotSpeeds,                      // live chassis speeds
                    robotRotation,                    // live rotation
                    Meters.of(0.8),                     // shooter height
                    MetersPerSecond.of(5.0),          // exit speed
                    Degrees.of(45.0)                  // launch angle
            );
            fuelOnFly.launch();
            SimulatedArena.getInstance().addGamePieceProjectile(fuelOnFly);
            System.out.println("projectiles=" + fuelProjectile.size());

        }
        lastshot = shoot;
        SimulatedArena.getInstance().simulationPeriodic();
        GamePieceProjectile.updateGamePieceProjectiles(SimulatedArena.getInstance(), fuelProjectile);

        Pose3d[] fuelsPoses = SimulatedArena.getInstance()
                .getGamePiecesArrayByType("RebuiltFuelOnField");
        ((StructArrayPublisher<Object>) (Object) fuelPublisher).accept( SimulatedArena.getInstance()
                .getGamePiecesByType("RebuiltFuelOnField")
                .stream().map(GamePiece::getPose3d).toArray());


    }
}