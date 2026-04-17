package frc.robot.subsystems

import com.ctre.phoenix6.CANBus
import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.CoastOut
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.MotorAlignmentValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsystems.projectile.Vector3
import frc.robot.subsystems.projectile.analyticInitialGuess
import frc.robot.util.ShotLookupTables
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.pow
import kotlin.math.sqrt

class ShooterSubsystem : SubsystemBase() {
    val shooterCANBus: CANBus = CANBus("Shooter CANivore")
    val defaultCANBus: CANBus = CANBus("Default Name")
    private val hoodMotor     = TalonFX(ShooterConstants.HOOD_MOTOR_ID, shooterCANBus)
    private val upperFlywheel = TalonFX(ShooterConstants.TOP_FLYWHEEL_MOTOR_ID, shooterCANBus)
    private val lowerFlywheel = TalonFX(ShooterConstants.BOTTOM_FLYWHEEL_MOTOR_ID, shooterCANBus)
    private val turretMotor   = TalonFX(ShooterConstants.TURRET_MOTOR_ID, defaultCANBus)
    private val turretEncoder = CANcoder(ShooterConstants.TURRET_ENCODER_ID, defaultCANBus)

    private val coastRequest = CoastOut()
    private val turretRequest   = MotionMagicTorqueCurrentFOC(0.0)        .withSlot(0)
    private val hoodRequest     = MotionMagicTorqueCurrentFOC(0.0)        .withSlot(0)
    private val flywheelRequest = VelocityTorqueCurrentFOC(0.0).withSlot(0)
    private val flywheelFollowRequest = Follower(ShooterConstants.TOP_FLYWHEEL_MOTOR_ID, MotorAlignmentValue.Opposed)

    private var turretGoalRot        = ShooterConstants.TURRET_HOME_ROT
    private var hoodGoalRot          = ShooterConstants.HOOD_HOME_ROT
    private var topFlywheelGoalRps    = 0.0
    private var bottomFlywheelGoalRps = 0.0
    val gearRatio = 11.0 / 5.0
    val upperCircumferenceM = ShooterConstants.UPPER_FLYWHEEL_DIAMETER_M * Math.PI
    val lowerCircumferenceM = ShooterConstants.LOWER_FLYWHEEL_DIAMETER_M * Math.PI

    val shooterOffset = Transform2d(-6.0 * 0.0254, 0.0, Rotation2d())
    val goalPos = Pose2d(182.11 * 0.0254, 158.84 * 0.0254, Rotation2d())

    private var hasValidShotSolution = false

    init {
        configTurret()
        configHood()
        configFlywheels()

        setTurretGoalRot(ShooterConstants.TURRET_HOME_ROT)
        setHoodGoalRot(ShooterConstants.HOOD_HOME_ROT)

        stopFlywheels()
    }

    private fun configTurret() {
        turretEncoder.configurator.apply(CANcoderConfiguration());

        val cfg = TalonFXConfiguration()

        cfg.MotorOutput = MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.CounterClockwise_Positive)

        cfg.Feedback = FeedbackConfigs()
            .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
            .withFeedbackRemoteSensorID(ShooterConstants.TURRET_ENCODER_ID)
            .withSensorToMechanismRatio(ShooterConstants.TURRET_PRIMARY_TO_MECHANISM_RATIO)
//            .withSensorToMechanismRatio(ShooterConstants.TURRET_SENSOR_TO_MECHANISM_RATIO)

        cfg.Slot0 = Slot0Configs()
            .withKS(ShooterConstants.TURRET_kS)
            .withKV(ShooterConstants.TURRET_kV)
            .withKA(ShooterConstants.TURRET_kA)
            .withKP(ShooterConstants.TURRET_kP)
            .withKD(ShooterConstants.TURRET_kD)
            .withKI(ShooterConstants.TURRET_kI)

        cfg.MotionMagic.MotionMagicCruiseVelocity = ShooterConstants.TURRET_CRUISE_VEL
        cfg.MotionMagic.MotionMagicAcceleration   = ShooterConstants.TURRET_ACCEL
        cfg.MotionMagic.MotionMagicJerk            = ShooterConstants.TURRET_JERK
        cfg.CurrentLimits.StatorCurrentLimit       = 40.0
        cfg.CurrentLimits.StatorCurrentLimitEnable = true

        turretMotor.configurator.apply(cfg)
    }

    private fun configHood() {
        val cfg = TalonFXConfiguration()

        cfg.MotorOutput = MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.Clockwise_Positive)

        cfg.Feedback = FeedbackConfigs()
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
            .withSensorToMechanismRatio(ShooterConstants.HOOD_GEAR_RATIO)

        cfg.Slot0 = Slot0Configs()
            .withKS(ShooterConstants.HOOD_kS)
            .withKV(ShooterConstants.HOOD_kV)
            .withKA(ShooterConstants.HOOD_kA)
            .withKP(ShooterConstants.HOOD_kP)
            .withKD(ShooterConstants.HOOD_kD)
            .withKG(ShooterConstants.HOOD_kG)
            .withKI(ShooterConstants.HOOD_kI)

        cfg.MotionMagic.MotionMagicCruiseVelocity = ShooterConstants.HOOD_CRUISE_VEL
        cfg.MotionMagic.MotionMagicAcceleration   = ShooterConstants.HOOD_ACCEL
        cfg.MotionMagic.MotionMagicJerk           = ShooterConstants.HOOD_JERK
        cfg.CurrentLimits.StatorCurrentLimit       = 60.0
        cfg.CurrentLimits.StatorCurrentLimitEnable = true

        hoodMotor.configurator.apply(cfg)
    }

    private fun configFlywheels() {
        val leftCfg = TalonFXConfiguration()
        leftCfg.MotorOutput = MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(InvertedValue.CounterClockwise_Positive)


        leftCfg.Slot0 = Slot0Configs()
            .withKV(ShooterConstants.FLYWHEEL_kV)
            .withKA(ShooterConstants.FLYWHEEL_kA)
            .withKS(ShooterConstants.FLYWHEEL_kS)
            .withKP(ShooterConstants.FLYWHEEL_kP)
            .withKI(ShooterConstants.FLYWHEEL_kI)
            .withKD(ShooterConstants.FLYWHEEL_kD)

//        leftCfg.MotionMagic.MotionMagicAcceleration = ShooterConstants.FLYWHEEL_ACCEL
        leftCfg.CurrentLimits.StatorCurrentLimit       = 80.0
        leftCfg.CurrentLimits.StatorCurrentLimitEnable = true
        upperFlywheel.configurator.apply(leftCfg)
//
//        val rightCfg = TalonFXConfiguration()
//        rightCfg.MotorOutput = MotorOutputConfigs()
//            .withNeutralMode(NeutralModeValue.Coast)
//            .withInverted(InvertedValue.Clockwise_Positive)
//        rightCfg.Slot0 = Slot0Configs()
//            .withKV(ShooterConstants.FLYWHEEL_kV)
//            .withKA(ShooterConstants.FLYWHEEL_kA)
//            .withKP(ShooterConstants.FLYWHEEL_kP)
//        rightCfg.MotionMagic.MotionMagicAcceleration = ShooterConstants.FLYWHEEL_ACCEL
//        rightCfg.CurrentLimits.StatorCurrentLimit       = 80.0
//        rightCfg.CurrentLimits.StatorCurrentLimitEnable = true
//        lowerFlywheel.configurator.apply(rightCfg)
    }

    fun setTurretGoalRot(rot: Double) {
        turretGoalRot = MathUtil.clamp(
            rot,
            ShooterConstants.TURRET_MIN_ROT,
            ShooterConstants.TURRET_MAX_ROT
        )
        turretMotor.setControl(turretRequest.withPosition(turretGoalRot))
    }

    fun setHoodGoalRot(rot: Double) {
        hoodMotor.setControl(hoodRequest.withPosition(rot))
    }

    fun setFlywheelSpeeds(rps: Double) {
        topFlywheelGoalRps = rps
        upperFlywheel.setControl(flywheelRequest.withVelocity(rps))
        lowerFlywheel.setControl(flywheelFollowRequest)
    }

    fun stopFlywheels() {
        upperFlywheel.setControl(coastRequest)
    }

    fun stopAiming() {
        hasValidShotSolution = false
        stopFlywheels()
    }

    fun homeTurret() = setTurretGoalRot(ShooterConstants.TURRET_HOME_ROT)

    fun homeHood() = setHoodGoalRot(ShooterConstants.HOOD_HOME_ROT)

    fun turretAtGoal(): Boolean =
        abs(turretMotor.position.valueAsDouble - turretGoalRot) < ShooterConstants.TURRET_TOLERANCE_ROT

    fun hoodAtGoal(): Boolean =
        abs(hoodMotor.position.valueAsDouble - hoodGoalRot) < ShooterConstants.HOOD_TOLERANCE_ROT

    fun flywheelsAtSpeed(): Boolean =
        abs(abs(upperFlywheel.velocity.valueAsDouble) - abs(topFlywheelGoalRps)) < ShooterConstants.FLYWHEEL_TOLERANCE_RPS &&
                abs(abs(lowerFlywheel.velocity.valueAsDouble) - abs(bottomFlywheelGoalRps)) < ShooterConstants.FLYWHEEL_TOLERANCE_RPS

    fun readyToFire(): Boolean = hasValidShotSolution && flywheelsAtSpeed() && hoodAtGoal()

    fun hasValidShot(): Boolean = hasValidShotSolution

    fun hoodDegreesToMechanismRotations(deg: Double): Double {
        return deg / 360.0 * ShooterConstants.HOOD_GEAR_RATIO
    }

    fun autoAimAndSpin(robotPos: Pose2d) {
        val shooterPos = robotPos + shooterOffset
        val distFromGoal = (goalPos - shooterPos).translation.norm
        SmartDashboard.putNumber("Shooter/distFromGoal", distFromGoal)
        val shot = ShotLookupTables.getShotForDistance(distFromGoal)
        SmartDashboard.putBoolean("Shooter/hasShot", shot != null)

        if (shot == null) {
            println("Shot not found")
            stopFlywheels()
            hasValidShotSolution = false
        } else {

            println("Has Shot")
            setFlywheelSpeeds(shot.rps)
            setHoodGoalRot(shot.hoodDeg)
            SmartDashboard.putNumber("Shooter/hoodGoalAngle", shot.hoodDeg)

            hasValidShotSolution = true
        }
    }

    override fun periodic() {
//        SmartDashboard.putNumber("Shooter/TopFlywheel/GoalRps",        topFlywheelGoalRps)
//        SmartDashboard.putNumber("Shooter/TopFlywheel/ClosedLoopRef",  upperFlywheel.closedLoopReference.valueAsDouble)
//        SmartDashboard.putNumber("Shooter/TopFlywheel/ClosedLoopOut",  upperFlywheel.closedLoopOutput.valueAsDouble)
//        SmartDashboard.putNumber("Shooter/TopFlywheel/RotorVel",       upperFlywheel.rotorVelocity.valueAsDouble)
//        SmartDashboard.putNumber("Shooter/TopFlywheel/MechanismVel",   upperFlywheel.velocity.valueAsDouble)

//        SmartDashboard.putNumber("Shooter/TopFlywheel/ClosedLoopError",upperFlywheel.closedLoopError.valueAsDouble)
//        SmartDashboard.putNumber("Shooter/TopFlywheel/StatorCurrent",  upperFlywheel.statorCurrent.valueAsDouble)
//        SmartDashboard.putNumber("Shooter/TopFlywheel/SupplyCurrent",  upperFlywheel.supplyCurrent.valueAsDouble)
        SmartDashboard.putNumber("Shooter/TopFlywheel/MotorVoltage",   upperFlywheel.motorVoltage.valueAsDouble)

//        SmartDashboard.putNumber("Shooter/Turret/GoalRot",         turretGoalRot)
//        SmartDashboard.putNumber("Shooter/Turret/ClosedLoopRef",   turretMotor.closedLoopReference.valueAsDouble)
//        SmartDashboard.putNumber("Shooter/Turret/ClosedLoopOut",   turretMotor.closedLoopOutput.valueAsDouble)
        SmartDashboard.putNumber("Shooter/Turret/EncoderAngle", turretEncoder.absolutePosition.valueAsDouble)
       SmartDashboard.putNumber("Shooter/Turret/MechanismPos",    turretMotor.position.valueAsDouble)

//        SmartDashboard.putNumber("Shooter/Turret/ClosedLoopError", turretMotor.closedLoopError.valueAsDouble)
        SmartDashboard.putNumber("Shooter/Turret/StatorCurrent",   turretMotor.torqueCurrent.valueAsDouble)
        SmartDashboard.putNumber("Shooter/Turret/SupplyCurrent",   turretMotor.supplyCurrent.valueAsDouble)
        SmartDashboard.putNumber("Shooter/Turret/MotorVoltage",    turretMotor.motorVoltage.valueAsDouble)
        SmartDashboard.putBoolean("Shooter/Turret/TurretAlive", turretMotor.isAlive)

        SmartDashboard.putNumber("Shooter/FlywheelVelocityMps", ShooterConstants.ntFlywheelVelocity.get())
        SmartDashboard.putNumber("Shooter/HoodAngleDeg",        ShooterConstants.ntHoodAngleDeg.get())
        SmartDashboard.putNumber("Shooter/TurretAngleDeg",      ShooterConstants.ntTurretAngleDeg.get())

//        SmartDashboard.putNumber("Shooter/TurretGoalRot", turretGoalRot)
        SmartDashboard.putNumber("Shooter/TurretPosRot",  turretMotor.position.valueAsDouble)

//        SmartDashboard.putNumber("Shooter/HoodGoalRot", hoodGoalRot)
        SmartDashboard.putNumber("Shooter/HoodPosRot",  hoodMotor.position.valueAsDouble)

        SmartDashboard.putNumber("Shooter/TopFlywheelGoalRPS", topFlywheelGoalRps)
        SmartDashboard.putNumber("Shooter/TopFlywheelRPS",     upperFlywheel.velocity.valueAsDouble)
        SmartDashboard.putBoolean("Shooter/TopFlywheelAlive",     upperFlywheel.isAlive)

        SmartDashboard.putNumber("Shooter/BottomFlywheelGoalRPS", bottomFlywheelGoalRps)
        SmartDashboard.putNumber("Shooter/BottomFlywheelRPS",     lowerFlywheel.velocity.valueAsDouble)

//        SmartDashboard.putBoolean("Shooter/TurretAtGoal",     turretAtGoal())
//        SmartDashboard.putBoolean("Shooter/HoodAtGoal",       hoodAtGoal())
//        SmartDashboard.putBoolean("Shooter/FlywheelsAtSpeed", flywheelsAtSpeed())
//        SmartDashboard.putBoolean("Shooter/ReadyToFire",      readyToFire())
    }
}