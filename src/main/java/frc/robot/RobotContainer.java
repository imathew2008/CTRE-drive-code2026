// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.questNav.QuestNavSubsystem;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionSim;
import frc.robot.subsystems.vision.LimelightMeasurementSource;
import frc.robot.FieldZone;
import frc.robot.BumpCorrection;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import java.io.File;

import static edu.wpi.first.units.Units.*;
import static frc.robot.HelperFunctionsKt.*;

public class RobotContainer {
    private static final double deadBand = 0.05;
//    private final BumpCorrection bumpCorrection = new BumpCorrection(7.0, 9.0);
//    private final PIDController bumpHeadingPid = new PIDController(8.0, 0.0, 0.35);
//
//    private Rotation2d lastBumpTargetHeading = new Rotation2d();
//    private boolean lastInBump = false;

   public final VisionEstimation visionEst;
    public final VisionSim visionSim;
    public final LimelightMeasurementSource limelightSource;
    public final QuestNavSubsystem questNavSubsystem;

    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight =
            new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    AprilTagFieldLayout fieldLayout;

    public RobotContainer() {
        configureBindings();

//        bumpHeadingPid.enableContinuousInput(-Math.PI, Math.PI);

        // Camera pose relative to robot (fill in real values later)
        Transform3d robotToCameraOne = new Transform3d(
                new Translation3d(-0.265, -0.366, 0.502),
                new Rotation3d(0.0, 0.0, Math.toRadians(45.0))
        );
        Transform3d robotToCameraTwo = new Transform3d(
                new Translation3d(-0.265, 0.366, 0.502),
                new Rotation3d(0.0, 0.0, Math.toRadians(-45.0))
        );

        try {
            fieldLayout = new AprilTagFieldLayout(
                    new File(Filesystem.getDeployDirectory(), "2026-rebuilt-welded.json").toPath());
        } catch (Exception e) {
            throw new RuntimeException("Failed to load AprilTag layout", e);
        }

        this.visionSim = new VisionSim(
                "cameraOne",
                "cameraTwo",
                robotToCameraOne,
                robotToCameraTwo,
                fieldLayout
        );

        this.limelightSource = new LimelightMeasurementSource(
                visionSim.getCameraOne(),
                visionSim.getCameraTwo(),
                fieldLayout,
                robotToCameraOne,
                robotToCameraTwo
        );

        this.visionEst = new VisionEstimation(drivetrain);
        this.questNavSubsystem = new QuestNavSubsystem(drivetrain);

        drivetrain.resetPose(new Pose2d(3, 3, new Rotation2d()));
    }

    public boolean shooting() { return joystick.a().getAsBoolean(); }

    private void configureBindings() {
//        drivetrain.setDefaultCommand(
//                drivetrain.applyRequest(() -> {
//                    Pose2d currentPose = drivetrain.getState().Pose;
//
//                    Translation2d translation = new Translation2d(
//                            -joystick.getLeftY() * MaxSpeed,
//                            -joystick.getLeftX() * MaxSpeed
//                    );
//
//                    boolean inBump = bumpCorrection.isInBump(currentPose.getX());
//
//                    if (inBump && translation.getNorm() > 10.0) {
//                        translation = translation.times(10.0 / translation.getNorm());
//                    }
//
//                    double vx = translation.getX();
//                    double vy = translation.getY();
//
//                    double omega;
//                    if (inBump) {
//                        Rotation2d targetHeading = bumpCorrection.getTargetHeading(
//                                currentPose.getX(),
//                                currentPose.getRotation()
//                        );
//
//                        double dt = 0.02;
//
//                        if (!lastInBump) {
//                            lastBumpTargetHeading = targetHeading;
//                        }
//
//                        double targetAngularVelocity = MathUtil.angleModulus(
//                                targetHeading.getRadians() - lastBumpTargetHeading.getRadians()
//                        ) / dt;
//
//                        double pidOmega = bumpHeadingPid.calculate(
//                                currentPose.getRotation().getRadians(),
//                                targetHeading.getRadians()
//                        );
//
//                        omega = targetAngularVelocity + pidOmega;
//                        omega = MathUtil.clamp(omega, -MaxAngularRate, MaxAngularRate);
//
//                        lastBumpTargetHeading = targetHeading;
//                    } else {
//                        omega = -joystick.getRightX() * MaxAngularRate;
//                    }
//
//                    lastInBump = inBump;
//
//                    return drive.withVelocityX(vx)
//                            .withVelocityY(vy)
//                            .withRotationalRate(omega);
//                })
//        );
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> {
                    double x   = -joystick.getLeftY();
                    double y   = -joystick.getLeftX();
                    double rot = -joystick.getRightX();

                    double[] xy  = applyCircularDeadband(x, y, deadBand);
                    xy           = squareVectorKeepDirection(xy[0], xy[1]);
                    double omega = squareKeepSign(applyDeadband1D(rot, deadBand));

                    return drive
                            .withVelocityX(xy[0] * MaxSpeed)
                            .withVelocityY(xy[1] * MaxSpeed)
                            .withRotationalRate(omega * MaxAngularRate);
                }));

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b()
                .whileTrue(drivetrain.applyRequest(
                        () -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

        joystick.pov(0)
                .whileTrue(drivetrain.applyRequest(
                        () -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
        joystick.pov(180)
                .whileTrue(drivetrain.applyRequest(
                        () -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }
}