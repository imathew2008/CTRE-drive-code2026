package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.DataClassesKt.zoneName;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.sensors.LimelightSubsystem;
import frc.robot.sensors.QuestNavSubsystem;
import frc.robot.sensors.VisionEstimation;
import frc.robot.subsystems.projectile.Vector3;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.zones.StableZoneLookup;
import frc.robot.zones.ZoneLookup;

public class RobotContainer {
    private static final double deadBand = 0.05;
    private static final double triggerThreshold = 0.3;

    private final ZoneLookup zoneLookup =
            new ZoneLookup(
                    FilteredFieldMap.WIDTH,
                    FilteredFieldMap.HEIGHT,
                    FilteredFieldMap.CELL_SIZE_INCHES,
                    FilteredFieldMap.INSTANCE.getZONES()
            );
    final StableZoneLookup stableZoneLookup = new StableZoneLookup(zoneLookup);

    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(0.0)
            .withRotationalDeadband(0.0)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final CommandXboxController driverController      = new CommandXboxController(0);
    private final CommandXboxController manipulatorController = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final IntakeSubsystem intake   = new IntakeSubsystem();
    private final IndexSubsystem indexer   = new IndexSubsystem();
    private final ShooterSubsystem shooter = new ShooterSubsystem();
    public final QuestNavSubsystem questNavSubsystem;
    public final LimelightSubsystem limelightSubsystem;

    private final SendableChooser<Command> autoChooser;

    StructPublisher<Pose2d> posEst = NetworkTableInstance.getDefault().getStructTopic("PosEst", Pose2d.struct)
            .publish();

    public RobotContainer() {
        configureBindings();

        this.questNavSubsystem = new QuestNavSubsystem();
        this.limelightSubsystem = new LimelightSubsystem(drivetrain);

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Chooser", autoChooser);
        autoChooser.setDefaultOption("Pathfind to point", goToPoint());
    }

    private Vector3 getCurrentShooterPosition() {
        var pose = drivetrain.getState().Pose;

        double shooterOffsetX = 0.0;
        double shooterOffsetY = 0.0;
        double shooterHeightMeters = 0.4318;

        double cos = pose.getRotation().getCos();
        double sin = pose.getRotation().getSin();

        double shooterX = pose.getX() + shooterOffsetX * cos - shooterOffsetY * sin;
        double shooterZ = pose.getY() + shooterOffsetX * sin + shooterOffsetY * cos;

        return new Vector3(shooterX, shooterHeightMeters, shooterZ);
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> {
                    double x   = -driverController.getLeftY();
                    double y   = -driverController.getLeftX();
                    double rot = -driverController.getRightX();

                    double[] xy = applyCircularDeadband(x, y, deadBand);
                    xy = squareVectorKeepDirection(xy[0], xy[1]);
                    double omega = squareKeepSign(applyDeadband1D(rot, deadBand));

                    return drive
                            .withVelocityX(xy[0] * MaxSpeed)
                            .withVelocityY(xy[1] * MaxSpeed)
                            .withRotationalRate(omega * MaxAngularRate);
                }));

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        //x-lock
        driverController.x().whileTrue(
                drivetrain.applyRequest(() -> brake));

        //hopper extrusion
        new Trigger(() -> driverController.getLeftTriggerAxis() > triggerThreshold)
                .onTrue(Commands.runOnce(intake::requestToggle, intake));

        //intake rollers
        new Trigger(() -> driverController.getRightTriggerAxis() > triggerThreshold)
                .whileTrue(Commands.run(intake::requestRunIntake, intake))
                .onFalse(Commands.runOnce(intake::requestStopIntake, intake));

        //run trigger + indexer
        new Trigger(() -> manipulatorController.getRightTriggerAxis() > triggerThreshold)
                .whileTrue(Commands.run(indexer::requestRunSystem, indexer))
                .onFalse(Commands.runOnce(indexer::requestStopSystem, indexer));

        //reverse trigger + indexer
        manipulatorController.a().whileTrue(Commands.runOnce(indexer::toggleModifier, indexer))
                .onFalse(Commands.runOnce(indexer::toggleModifier, indexer));

        //aim and ramp up
        new Trigger (() -> manipulatorController.getLeftTriggerAxis() > triggerThreshold)
                .whileTrue(Commands.run(shooter::applyNetworkTableSetpoints, shooter))
                .onFalse(Commands.runOnce(shooter::stopAiming, shooter));

        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        driverController.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        int currentZone = stableZoneLookup.getStableZone(drivetrain.getState().Pose);

        SmartDashboard.putString("Zone/Current Zone", zoneName(currentZone));
        posEst.set(VisionEstimation.getEstimatedPose2d());
    }

    public Command getAutonomousCommand() {

        return autoChooser.getSelected();
    }

    public Command goToPoint() {
        return AutoBuilder.pathfindToPose(
                        new Pose2d(3.6, 7.371, Rotation2d.fromDegrees(0)),
                        new PathConstraints(
                                2.0, 2.0,
                                Units.degreesToRadians(360),
                                Units.degreesToRadians(540)
                        ), 0.0);
    }

//    Command autoCommand() {
//        Commands.sequence(
//        shooter.setFlywheelSpeeds(3830.2481, 3830.2481);
//        if(shooter.flywheelsAtSpeed()) {
//            indexer.runIndexer();
//            indexer.runFeeder();
//        }
//        shooter.
//        )

    public static double[] applyCircularDeadband(double x, double y, double deadband) {
        double mag = Math.sqrt(x * x + y * y);
        if (mag < deadband) {
            return new double[]{0.0, 0.0};
        }
        double scaledMag = (mag - deadband) / (1.0 - deadband);
        double scale = scaledMag / mag;

        return new double[]{x * scale, y * scale};
    }

    public static double applyDeadband1D(double value, double deadband) {
        if (Math.abs(value) <= deadband) {
            return 0.0;
        }
        double scaled = (Math.abs(value) - deadband) / (1.0 - deadband);
        return Math.copySign(scaled, value);
    }

    public static double squareKeepSign(double x) {
        return Math.copySign(x * x, x);
    }

    public static double[] squareVectorKeepDirection(double x, double y) {
        double mag = Math.hypot(x, y);
        if (mag <= 1e-9) return new double[] {0.0, 0.0};
        return new double[] {x * mag, y * mag};
    }
}