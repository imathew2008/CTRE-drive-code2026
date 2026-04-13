package frc.robot.subsystems

import com.ctre.phoenix6.CANBus
import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsystems.projectile.Vector3
import frc.robot.subsystems.projectile.analyticInitialGuess
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.sqrt

class ShooterSubsystem : SubsystemBase() {
    val canBus: CANBus = CANBus("Default Name", "./logs/example.hoot")
    private val hoodMotor     = TalonFX(ShooterConstants.HOOD_MOTOR_ID, canBus)
    private val turretMotor   = TalonFX(ShooterConstants.TURRET_MOTOR_ID, canBus)
    private val upperFlywheel = TalonFX(ShooterConstants.LEFT_FLYWHEEL_MOTOR_ID, canBus)
    private val lowerFlywheel = TalonFX(ShooterConstants.RIGHT_FLYWHEEL_MOTOR_ID, canBus)

    private val turretEncoder = CANcoder(ShooterConstants.TURRET_ENCODER_ID, canBus)
    private val hoodEncoder   = CANcoder(ShooterConstants.HOOD_ENCODER_ID, canBus)

    private val turretRequest   = MotionMagicTorqueCurrentFOC(0.0)        .withSlot(0)
    private val hoodRequest     = MotionMagicTorqueCurrentFOC(0.0)        .withSlot(0)
    private val flywheelRequest = MotionMagicVelocityTorqueCurrentFOC(0.0).withSlot(0)

    private var turretGoalRot        = ShooterConstants.TURRET_HOME_ROT
    private var hoodGoalRot          = ShooterConstants.HOOD_HOME_ROT
    private var topFlywheelGoalRps    = 0.0
    private var bottomFlywheelGoalRps = 0.0
    val gearRatio = 11.0 / 5.0
    val upperCircumferenceM = ShooterConstants.UPPER_FLYWHEEL_DIAMETER_M * Math.PI
    val lowerCircumferenceM = ShooterConstants.LOWER_FLYWHEEL_DIAMETER_M * Math.PI

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
        val encoderCfg = CANcoderConfiguration()
        turretEncoder.configurator.apply(encoderCfg)

        val cfg = TalonFXConfiguration()

        cfg.MotorOutput = MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.CounterClockwise_Positive)

        cfg.Feedback = FeedbackConfigs()
            .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
            .withFeedbackRemoteSensorID(ShooterConstants.TURRET_ENCODER_ID)
//            .withRotorToSensorRatio(ShooterConstants.TURRET_ROTOR_TO_SENSOR_RATIO)
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
        val encoderCfg = CANcoderConfiguration()
        hoodEncoder.configurator.apply(encoderCfg)

        val cfg = TalonFXConfiguration()

        cfg.MotorOutput = MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.CounterClockwise_Positive)

        cfg.Feedback = FeedbackConfigs()
            .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
            .withFeedbackRemoteSensorID(ShooterConstants.HOOD_ENCODER_ID)
//            .withRotorToSensorRatio(ShooterConstants.HOOD_ROTOR_TO_SENSOR_RATIO)
//            .withSensorToMechanismRatio(ShooterConstants.HOOD_SENSOR_TO_MECHANISM_RATIO)

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

        cfg.CurrentLimits.StatorCurrentLimit       = 40.0
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

        leftCfg.MotionMagic.MotionMagicAcceleration = ShooterConstants.FLYWHEEL_ACCEL
        leftCfg.CurrentLimits.StatorCurrentLimit       = 80.0
        leftCfg.CurrentLimits.StatorCurrentLimitEnable = true
        upperFlywheel.configurator.apply(leftCfg)

        val rightCfg = TalonFXConfiguration()
        rightCfg.MotorOutput = MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(InvertedValue.Clockwise_Positive)
        rightCfg.Slot0 = Slot0Configs()
            .withKV(ShooterConstants.FLYWHEEL_kV)
            .withKA(ShooterConstants.FLYWHEEL_kA)
            .withKP(ShooterConstants.FLYWHEEL_kP)
        rightCfg.MotionMagic.MotionMagicAcceleration = ShooterConstants.FLYWHEEL_ACCEL
        rightCfg.CurrentLimits.StatorCurrentLimit       = 80.0
        rightCfg.CurrentLimits.StatorCurrentLimitEnable = true
        lowerFlywheel.configurator.apply(rightCfg)
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
        hoodGoalRot = MathUtil.clamp(
            rot,
            ShooterConstants.HOOD_MIN_ROT,
            ShooterConstants.HOOD_MAX_ROT
        )
        hoodMotor.setControl(hoodRequest.withPosition(hoodGoalRot))
    }

    fun setFlywheelSpeeds(topRps: Double, bottomRps: Double) {
        topFlywheelGoalRps    = topRps
        bottomFlywheelGoalRps = bottomRps
        upperFlywheel.setControl(flywheelRequest.withVelocity(topFlywheelGoalRps))
        lowerFlywheel.setControl(flywheelRequest.withVelocity(bottomFlywheelGoalRps))
    }

    fun runFlywheels() {
        setFlywheelSpeeds(
            ShooterConstants.TOP_FLYWHEEL_TARGET_RPS,
            ShooterConstants.BOTTOM_FLYWHEEL_TARGET_RPS
        )
    }

    fun stopFlywheels() {
        setFlywheelSpeeds(0.0, 0.0)
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

    fun readyToFire(): Boolean = flywheelsAtSpeed()

    fun hasValidShot(): Boolean = hasValidShotSolution

    fun aimAndSpinFromVector(velocity: Vector3) {
        val speed = sqrt(
            velocity.x * velocity.x +
                    velocity.y * velocity.y +
                    velocity.z * velocity.z
        )

        val horizontalSpeed = sqrt(
            velocity.x * velocity.x +
                    velocity.z * velocity.z
        )

        val hoodAngleRad   = atan2(velocity.y, horizontalSpeed)
        val turretAngleRad = atan2(velocity.z, velocity.x)

//        setHoodGoalRot(hoodRadiansToMechanismRotations(hoodAngleRad))
//        setTurretGoalRot(turretRadiansToMechanismRotations(turretAngleRad))
//        val topRps = projectileSpeedToTopFlywheelRps(speed)
//        val bottomRps = projectileSpeedToBottomFlywheelRps(speed)
//        setFlywheelSpeeds(topRps, bottomRps)

        hasValidShotSolution = true
    }

    //need to add actual sim stuff
    fun updateShotFromPosition(shooterPosition: Vector3): Boolean {
        val solution = analyticInitialGuess(
            p0 = shooterPosition,
            goal = Vector3(3.05, 1.8, 0.0),
            desiredImpactAngle = Math.toRadians(-45.0),
            g = 9.81
        )

        if (solution == null) {
            hasValidShotSolution = false
            stopFlywheels()
            return false
        }

        aimAndSpinFromVector(solution)
        return true
    }

    fun applyNetworkTableSetpoints() {
        val velocityMps = ShooterConstants.ntFlywheelVelocity.get()
        val hoodDeg     = ShooterConstants.ntHoodAngleDeg.get()
        val turretDeg   = ShooterConstants.ntTurretAngleDeg.get()

        val upperRps = (velocityMps / upperCircumferenceM) * gearRatio
        val lowerRps = (velocityMps / lowerCircumferenceM) * gearRatio

//        val hoodRot   = hoodDeg   / 360.0 * ShooterConstants.HOOD_SENSOR_TO_MECHANISM_RATIO
//        val turretRot = turretDeg / 360.0 * ShooterConstants.TURRET_SENSOR_TO_MECHANISM_RATIO

        val hoodRot   = hoodDeg   / 360.0 * 1.0
        val turretRot = turretDeg / 360.0 * 1.0

        setFlywheelSpeeds(upperRps, lowerRps)
        setHoodGoalRot(hoodRot)
        setTurretGoalRot(turretRot)
    }

    override fun periodic() {
        // What we're commanding
        SmartDashboard.putNumber("Shooter/TopFlywheel/GoalRps",        topFlywheelGoalRps)
        SmartDashboard.putNumber("Shooter/TopFlywheel/ClosedLoopRef",  upperFlywheel.closedLoopReference.valueAsDouble)
        SmartDashboard.putNumber("Shooter/TopFlywheel/ClosedLoopOut",  upperFlywheel.closedLoopOutput.valueAsDouble)

// What the motor actually sees
        SmartDashboard.putNumber("Shooter/TopFlywheel/RotorVel",       upperFlywheel.rotorVelocity.valueAsDouble)
        SmartDashboard.putNumber("Shooter/TopFlywheel/MechanismVel",   upperFlywheel.velocity.valueAsDouble)

// Error and output
        SmartDashboard.putNumber("Shooter/TopFlywheel/ClosedLoopError",upperFlywheel.closedLoopError.valueAsDouble)
        SmartDashboard.putNumber("Shooter/TopFlywheel/StatorCurrent",  upperFlywheel.statorCurrent.valueAsDouble)
        SmartDashboard.putNumber("Shooter/TopFlywheel/SupplyCurrent",  upperFlywheel.supplyCurrent.valueAsDouble)
        SmartDashboard.putNumber("Shooter/TopFlywheel/MotorVoltage",   upperFlywheel.motorVoltage.valueAsDouble)

        SmartDashboard.putNumber("Shooter/Turret/GoalRot",         turretGoalRot)
        SmartDashboard.putNumber("Shooter/Turret/ClosedLoopRef",   turretMotor.closedLoopReference.valueAsDouble)
        SmartDashboard.putNumber("Shooter/Turret/ClosedLoopOut",   turretMotor.closedLoopOutput.valueAsDouble)

       SmartDashboard.putNumber("Shooter/Turret/MechanismPos",    turretMotor.position.valueAsDouble)

        SmartDashboard.putNumber("Shooter/Turret/ClosedLoopError", turretMotor.closedLoopError.valueAsDouble)
        SmartDashboard.putNumber("Shooter/Turret/StatorCurrent",   turretMotor.statorCurrent.valueAsDouble)
        SmartDashboard.putNumber("Shooter/Turret/SupplyCurrent",   turretMotor.supplyCurrent.valueAsDouble)
        SmartDashboard.putNumber("Shooter/Turret/MotorVoltage",    turretMotor.motorVoltage.valueAsDouble)

        SmartDashboard.putNumber("Shooter/FlywheelVelocityMps", ShooterConstants.ntFlywheelVelocity.get())
        SmartDashboard.putNumber("Shooter/HoodAngleDeg",        ShooterConstants.ntHoodAngleDeg.get())
        SmartDashboard.putNumber("Shooter/TurretAngleDeg",      ShooterConstants.ntTurretAngleDeg.get())

        SmartDashboard.putNumber("Shooter/TurretGoalRot", turretGoalRot)
        SmartDashboard.putNumber("Shooter/TurretPosRot",  turretMotor.position.valueAsDouble)

        SmartDashboard.putNumber("Shooter/HoodGoalRot", hoodGoalRot)
        SmartDashboard.putNumber("Shooter/HoodPosRot",  hoodMotor.position.valueAsDouble)

        SmartDashboard.putNumber("Shooter/TopFlywheelGoalRPS", topFlywheelGoalRps)
        SmartDashboard.putNumber("Shooter/TopFlywheelRPS",     upperFlywheel.velocity.valueAsDouble)
        SmartDashboard.putBoolean("Shooter/TopFlywheelAlive",     upperFlywheel.isAlive)

        SmartDashboard.putNumber("Shooter/BottomFlywheelGoalRPS", bottomFlywheelGoalRps)
        SmartDashboard.putNumber("Shooter/BottomFlywheelRPS",     lowerFlywheel.velocity.valueAsDouble)

        SmartDashboard.putBoolean("Shooter/TurretAtGoal",     turretAtGoal())
        SmartDashboard.putBoolean("Shooter/HoodAtGoal",       hoodAtGoal())
        SmartDashboard.putBoolean("Shooter/FlywheelsAtSpeed", flywheelsAtSpeed())
        SmartDashboard.putBoolean("Shooter/ReadyToFire",      readyToFire())
    }
}