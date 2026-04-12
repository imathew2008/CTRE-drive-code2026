package frc.robot.subsystems

import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.VelocityVoltage
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
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.sqrt

class ShooterSubsystem : SubsystemBase() {
    private val hoodMotor = TalonFX(ShooterConstants.HOOD_MOTOR_ID)
    private val turretMotor = TalonFX(ShooterConstants.TURRET_MOTOR_ID)
    private val leftFlywheel = TalonFX(ShooterConstants.LEFT_FLYWHEEL_MOTOR_ID)
    private val rightFlywheel = TalonFX(ShooterConstants.RIGHT_FLYWHEEL_MOTOR_ID)

    private val turretEncoder = CANcoder(ShooterConstants.TURRET_ENCODER_ID)
    private val hoodEncoder = CANcoder(ShooterConstants.HOOD_ENCODER_ID)

    private val turretRequest = MotionMagicVoltage(0.0).withSlot(0)
    private val hoodRequest = MotionMagicVoltage(0.0).withSlot(0)
    private val flywheelRequest = VelocityVoltage(0.0).withSlot(0)

    private var turretGoalRot = ShooterConstants.TURRET_HOME_ROT
    private var hoodGoalRot = ShooterConstants.HOOD_HOME_ROT
    private var topFlywheelGoalRps = 0.0
    private var bottomFlywheelGoalRps = 0.0

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
            .withRotorToSensorRatio(ShooterConstants.TURRET_ROTOR_TO_SENSOR_RATIO)
            .withSensorToMechanismRatio(ShooterConstants.TURRET_SENSOR_TO_MECHANISM_RATIO)

        cfg.Slot0 = Slot0Configs()
            .withKS(ShooterConstants.TURRET_kS)
            .withKV(ShooterConstants.TURRET_kV)
            .withKA(ShooterConstants.TURRET_kA)
            .withKP(ShooterConstants.TURRET_kP)
            .withKI(ShooterConstants.TURRET_kI)
            .withKD(ShooterConstants.TURRET_kD)

        cfg.MotionMagic.MotionMagicCruiseVelocity = ShooterConstants.TURRET_CRUISE_VEL
        cfg.MotionMagic.MotionMagicAcceleration = ShooterConstants.TURRET_ACCEL
        cfg.MotionMagic.MotionMagicJerk = ShooterConstants.TURRET_JERK

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
            .withRotorToSensorRatio(ShooterConstants.HOOD_ROTOR_TO_SENSOR_RATIO)
            .withSensorToMechanismRatio(ShooterConstants.HOOD_SENSOR_TO_MECHANISM_RATIO)

        cfg.Slot0 = Slot0Configs()
            .withKS(ShooterConstants.HOOD_kS)
            .withKV(ShooterConstants.HOOD_kV)
            .withKA(ShooterConstants.HOOD_kA)
            .withKP(ShooterConstants.HOOD_kP)
            .withKI(ShooterConstants.HOOD_kI)
            .withKD(ShooterConstants.HOOD_kD)
            .withKG(ShooterConstants.HOOD_kG)

        cfg.MotionMagic.MotionMagicCruiseVelocity = ShooterConstants.HOOD_CRUISE_VEL
        cfg.MotionMagic.MotionMagicAcceleration = ShooterConstants.HOOD_ACCEL
        cfg.MotionMagic.MotionMagicJerk = ShooterConstants.HOOD_JERK

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
        leftFlywheel.configurator.apply(leftCfg)

        val rightCfg = TalonFXConfiguration()
        rightCfg.MotorOutput = MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(InvertedValue.Clockwise_Positive)
        rightCfg.Slot0 = Slot0Configs()
            .withKV(ShooterConstants.FLYWHEEL_kV)
            .withKA(ShooterConstants.FLYWHEEL_kA)
        rightFlywheel.configurator.apply(rightCfg)
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
        topFlywheelGoalRps = topRps
        bottomFlywheelGoalRps = bottomRps

        leftFlywheel.setControl(flywheelRequest.withVelocity(topFlywheelGoalRps))
        rightFlywheel.setControl(flywheelRequest.withVelocity(bottomFlywheelGoalRps))
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
//        homeTurret()
//        homeHood()
        stopFlywheels()
    }

    fun homeTurret() = setTurretGoalRot(ShooterConstants.TURRET_HOME_ROT)

    fun homeHood() = setHoodGoalRot(ShooterConstants.HOOD_HOME_ROT)

    fun turretAtGoal(): Boolean =
        abs(turretMotor.position.valueAsDouble - turretGoalRot) < ShooterConstants.TURRET_TOLERANCE_ROT

    fun hoodAtGoal(): Boolean =
        abs(hoodMotor.position.valueAsDouble - hoodGoalRot) < ShooterConstants.HOOD_TOLERANCE_ROT

    fun flywheelsAtSpeed(): Boolean =
        abs(abs(leftFlywheel.velocity.valueAsDouble) - abs(topFlywheelGoalRps)) < ShooterConstants.FLYWHEEL_TOLERANCE_RPS &&
                abs(abs(rightFlywheel.velocity.valueAsDouble) - abs(bottomFlywheelGoalRps)) < ShooterConstants.FLYWHEEL_TOLERANCE_RPS

    fun readyToFire(): Boolean {
        return flywheelsAtSpeed()
    }

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

        val hoodAngleRad = atan2(velocity.y, horizontalSpeed)
        val turretAngleRad = atan2(velocity.z, velocity.x)

//        setHoodGoalRot(hoodRadiansToMechanismRotations(hoodAngleRad))
//        setTurretGoalRot(turretRadiansToMechanismRotations(turretAngleRad))
//
//        val topRps = projectileSpeedToTopFlywheelRps(speed)
//        val bottomRps = projectileSpeedToBottomFlywheelRps(speed)
//        setFlywheelSpeeds(topRps, bottomRps)

        hasValidShotSolution = true
    }

    fun updateShotFromPosition(
        shooterPosition: Vector3,
    ): Boolean {
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

//    private fun hoodRadiansToMechanismRotations(angleRad: Double): Double {
//        return ShooterConstants.HOOD_ZERO_ROT + angleRad / (2.0 * PI)
//    }
//
//    private fun turretRadiansToMechanismRotations(angleRad: Double): Double {
//        return ShooterConstants.TURRET_ZERO_ROT + angleRad / (2.0 * PI)
//    }
//
//    private fun projectileSpeedToTopFlywheelRps(speedMps: Double): Double {
//        return speedMps * ShooterConstants.TOP_FLYWHEEL_RPS_PER_MPS
//    }
//
//    private fun projectileSpeedToBottomFlywheelRps(speedMps: Double): Double {
//        return speedMps * ShooterConstants.BOTTOM_FLYWHEEL_RPS_PER_MPS
//    }

    override fun periodic() {
        SmartDashboard.putNumber("Shooter/TurretGoalRot", turretGoalRot)
        SmartDashboard.putNumber("Shooter/TurretPosRot", turretMotor.position.valueAsDouble)

        SmartDashboard.putNumber("Shooter/HoodGoalRot", hoodGoalRot)
        SmartDashboard.putNumber("Shooter/HoodPosRot", hoodMotor.position.valueAsDouble)

        SmartDashboard.putNumber("Shooter/TopFlywheelGoalRPS", topFlywheelGoalRps)
        SmartDashboard.putNumber("Shooter/TopFlywheelRPS", leftFlywheel.velocity.valueAsDouble)

        SmartDashboard.putNumber("Shooter/BottomFlywheelGoalRPS", bottomFlywheelGoalRps)
        SmartDashboard.putNumber("Shooter/BottomFlywheelRPS", rightFlywheel.velocity.valueAsDouble)

        SmartDashboard.putBoolean("Shooter/TurretAtGoal", turretAtGoal())
        SmartDashboard.putBoolean("Shooter/HoodAtGoal", hoodAtGoal())
        SmartDashboard.putBoolean("Shooter/FlywheelsAtSpeed", flywheelsAtSpeed())
        SmartDashboard.putBoolean("Shooter/HasValidShot", hasValidShotSolution)
        SmartDashboard.putBoolean("Shooter/ReadyToFire", readyToFire())
    }
}