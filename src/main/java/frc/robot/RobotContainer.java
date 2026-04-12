// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.questNav.QuestNavSubsystem;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionSim;
import frc.robot.subsystems.vision.LimelightMeasurementSource;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import com.pathplanner.lib.auto.AutoBuilder;
import frc.robot.zones.StableZoneLookup;
import frc.robot.zones.ZoneLookup;
import java.io.File;
import static edu.wpi.first.units.Units.*;
import static frc.robot.HelperFunctionsKt.*;

public class RobotContainer {
    public boolean shooting = false;
    private final ZoneLookup zoneLookup =
            new ZoneLookup(
                    FilteredFieldMap.WIDTH,
                    FilteredFieldMap.HEIGHT,
                    FilteredFieldMap.CELL_SIZE_INCHES,
                    FilteredFieldMap.INSTANCE.getZONES()
            );

    final StableZoneLookup stableZoneLookup = new StableZoneLookup(zoneLookup);

    private static final double deadBand = 0.05;
    private final BumpCorrection bumpCorrection = new BumpCorrection();
    private final PIDController bumpHeadingPid = new PIDController(10.0, 0.0, 0.35);

    private Rotation2d lastBumpTargetHeading = new Rotation2d();
    private boolean lastInBump = false;

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

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer() {
        NamedCommands.registerCommand("startShoot", startSimShoot());
        NamedCommands.registerCommand("stopShoot", stopSimShoot());
        NamedCommands.registerCommand("extendIntake", stopSimShoot());
        NamedCommands.registerCommand("intakeIntake", stopSimShoot());
        NamedCommands.registerCommand("runIntake", stopSimShoot());

//        autoChooser.setDefaultOption("Normal Auto", new PathPlannerAuto("auto-one"));
        autoChooser.setDefaultOption("Pathfind + Auto", getPathfindThenAuto());
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());

        bumpHeadingPid.enableContinuousInput(-Math.PI, Math.PI);

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
//        SmartDashboard.putData("Example Auto", new PathPlannerAuto("auto-one"));

//        SmartDashboard.putData("Pathfind to Start Pos", AutoBuilder.pathfindToPose(
//                new Pose2d(14.0, 6.5, Rotation2d.fromDegrees(0)),
//                new PathConstraints(
//                        4.0, 4.0,
//                        Units.degreesToRadians(360), Units.degreesToRadians(540)
//                ),
//                0
//        ));
//        SmartDashboard.putData("Pathfind to Bump Pos", AutoBuilder.pathfindToPose(
//                new Pose2d(2.15, 3.0, Rotation2d.fromDegrees(180)),
//                new PathConstraints(
//                        4.0, 4.0,
//                        Units.degreesToRadians(360), Units.degreesToRadians(540)
//                ),
//                0
//        ));

        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> {
                    double x   = -joystick.getLeftY();
                    double y   = -joystick.getLeftX();
                    double rot = -joystick.getRightX();

                    double[] xy  = applyCircularDeadband(x, y, deadBand);
                    xy           = squareVectorKeepDirection(xy[0], xy[1]);

                    Pose2d currentPose = drivetrain.getState().Pose;

                    Translation2d translation = new Translation2d(
                            xy[0] * MaxSpeed,
                            xy[1] * MaxSpeed
                    );

                    int currentZone = stableZoneLookup.getStableZone(currentPose);
                    boolean inBump = BumpZones.isBumpZone(currentZone);

                    if (inBump && translation.getNorm() > MaxSpeed * 0.5) {
                        translation = translation.times(MaxSpeed * 0.5 / translation.getNorm());
                    }

                    double vx = translation.getX();
                    double vy = translation.getY();

                    double omega;

                    if (inBump) {
                        double dt = 0.02;
                        double preferredYSign = Math.signum(vy);

                        Rotation2d targetHeading = bumpCorrection.getTargetHeading(
                                currentZone,
                                currentPose.getRotation(),
                                preferredYSign
                        );

                        if (!lastInBump) {
                            lastBumpTargetHeading = targetHeading;
                            bumpHeadingPid.reset();
                        }

                        double targetAngularVelocity = MathUtil.angleModulus(
                                        targetHeading.getRadians() -
                                                lastBumpTargetHeading.getRadians()) / dt;

                        double pidOmega = bumpHeadingPid.calculate(currentPose.getRotation().getRadians(),
                                        targetHeading.getRadians());

                        omega = targetAngularVelocity + pidOmega;
                        omega = MathUtil.clamp(omega, -MaxAngularRate, MaxAngularRate);

                        lastBumpTargetHeading = targetHeading;
                    } else {
                        omega = squareKeepSign(applyDeadband1D(rot, deadBand)) * MaxAngularRate;
                    }

                    lastInBump = inBump;

                    return drive
                            .withVelocityX(vx)
                            .withVelocityY(vy)
                            .withRotationalRate(omega);
                })
        );

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
        joystick.back() .and(joystick.y()) .whileTrue(drivetrain.sysIdDynamic    (Direction.kForward));
        joystick.back() .and(joystick.x()) .whileTrue(drivetrain.sysIdDynamic    (Direction.kReverse));
        joystick.start().and(joystick.y()) .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()) .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public Command startSimShoot() {
        shooting = true;
        return null;
    }

    public Command stopSimShoot() {
        shooting = false;
        return null;
    }

    public Command getPathfindThenAuto() {
        return Commands.sequence(
                AutoBuilder.pathfindToPose(
                        new Pose2d(3.6, 7.371, Rotation2d.fromDegrees(0)),
                        new PathConstraints(
                                2.0, 2.0,
                                Units.degreesToRadians(360),
                                Units.degreesToRadians(540)
                        ),
                        0.0
                ),
                new PathPlannerAuto("auto-one")
        );
    }
}