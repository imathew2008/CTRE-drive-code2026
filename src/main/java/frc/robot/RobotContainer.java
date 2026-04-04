package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.subsystems.IntakeConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.projectile.Vector3;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.IndexSubsystem;

public class RobotContainer {
    private static final double deadBand = 0.05;
    private static final double triggerThreshold = 0.3;

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

    public RobotContainer() {
        configureBindings();
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

        driverController.rightTrigger().whileTrue(
                drivetrain.applyRequest(() -> brake));

        manipulatorController.rightTrigger(triggerThreshold).whileTrue(
                Commands.runEnd(
                        () -> {
                            indexer.runIndexer();
                            indexer.runFeeder();
                        },
                        () -> {
                            indexer.stopIndexer();
                            indexer.stopFeeder();
                        }
                )
        );

//        driverController.rightTrigger(triggerThreshold).whileTrue(
//                Commands.run(
//                        intake::intakeActuation,
//                        intake
//                )
//        );
//
//        driverController.leftTrigger(triggerThreshold).whileTrue(
//                Commands.runEnd(
//                        () -> intake.setRollerVoltage(IntakeConstants.ROLLER_INTAKE_VOLTS),
//                        intake::stopRollers,
//                        intake
//                )
//        );
//
//        manipulatorController.leftTrigger(triggerThreshold).whileTrue(
//                Commands.runEnd(
//                        () -> shooter.updateShotFromPosition(getCurrentShooterPosition()),
//                        shooter::stopAimingAndSpinning,
//                        shooter
//                )
//        );
//
//        manipulatorController.leftTrigger(triggerThreshold).whileTrue(
//                Commands.runEnd(
//                        () -> shooter.setFlywheelSpeeds(3830.2481, 3830.2481),
//                        shooter::stopAiming,
//                        shooter
//                )
//        );
//
//        manipulatorController.rightTrigger(triggerThreshold).whileTrue(
//                Commands.runEnd(
//                        () -> {
//                            if (shooter.readyToFire()) {
//                                indexer.runIndexer();
//                                indexer.runFeeder();
//                            } else {
//                                indexer.stopIndexer();
//                                indexer.stopFeeder();
//                            }
//                        },
//                        () -> {
//                            indexer.stopIndexer();
//                            indexer.stopFeeder();
//                        },
//                        indexer
//                )
//        );
//
//        manipulatorController.a().whileTrue(
//                Commands.runEnd(
//                        () -> intake.setRollerVoltage(IntakeConstants.ROLLER_OUTTAKE_VOLTS),
//                        intake::stopRollers,
//                        intake
//                )
//        );
//
//        manipulatorController.b().whileTrue(
//                Commands.parallel(
//                        Commands.runEnd(
//                                indexer::runIndexerReverse,
//                                indexer::stopIndexer,
//                                indexer
//                        ),
//                        Commands.runEnd(
//                                indexer::runFeederReverse,
//                                indexer::stopFeeder,
//                                indexer
//                        )
//                )
//        );
//
//        manipulatorController.leftTrigger(triggerThreshold).whileTrue(
//                Commands.runEnd(
//                        shooter::runFlywheels,
//                        shooter::stopFlywheels,
//                        shooter
//                )
//        );

        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        driverController.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
//        Commands.run(() -> shooter.setFlywheelSpeeds(3830.2481, 3830.2481))
//                .withDeadline(Commands.waitUntil(shooter::flywheelsAtSpeed).andThen(Commands.run(() -> {
//                    indexer.runIndexer();
//                    indexer.runFeeder();
//                }).withTimeout(5))).andThen(()-> Commands.runOnce(() -> {
//                    indexer.stopFeeder();
//                    indexer.stopIndexer();
//                }));
//        if(shooter.flywheelsAtSpeed()) {
//            indexer.runIndexer();
//            indexer.runFeeder();
//        }
        return null;
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
}