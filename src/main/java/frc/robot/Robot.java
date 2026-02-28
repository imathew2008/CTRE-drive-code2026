// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;

public class Robot extends TimedRobot {
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
        robotContainer = new RobotContainer();
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
    }

    @Override
    public void teleopPeriodic() {
        StructPublisher<Pose2d> pos = NetworkTableInstance.getDefault().getStructTopic("PredictedPos", Pose2d.struct)
                .publish();
        DoublePublisher gyroPos = NetworkTableInstance.getDefault().getDoubleTopic("Pigeon2 Pos").publish();
        DoublePublisher posx = NetworkTableInstance.getDefault().getDoubleTopic("pos x").publish();
        DoublePublisher posy = NetworkTableInstance.getDefault().getDoubleTopic("pos y").publish();
        DoublePublisher rot = NetworkTableInstance.getDefault().getDoubleTopic("rotation").publish();
        DoublePublisher realRot = NetworkTableInstance.getDefault().getDoubleTopic("realRot").publish();

    }

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
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

    }
}