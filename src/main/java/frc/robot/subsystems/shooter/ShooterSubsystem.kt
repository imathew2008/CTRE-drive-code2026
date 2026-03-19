package frc.robot.subsystems

import com.ctre.phoenix6.configs.*
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.NeutralOut
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import kotlin.math.abs

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
    private val neutralRequest = NeutralOut()

    private var turretGoalRot = ShooterConstants.TURRET_HOME_ROT
    private var hoodGoalRot = ShooterConstants.HOOD_HOME_ROT
    private var topFlywheelGoalRps = 0.0
    private var bottomFlywheelGoalRps = 0.0

    init {
        configTurret()
        configHood()
        configFlywheels()

        setTurretGoalRot(ShooterConstants.TURRET_HOME_ROT)
        setHoodGoalRot(ShooterConstants.HOOD_HOME_ROT)
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
            .withKS(ShooterConstants.FLYWHEEL_kS)
            .withKV(ShooterConstants.FLYWHEEL_kV)
            .withKA(ShooterConstants.FLYWHEEL_kA)
            .withKP(ShooterConstants.FLYWHEEL_kP)
            .withKI(ShooterConstants.FLYWHEEL_kI)
            .withKD(ShooterConstants.FLYWHEEL_kD)
        leftFlywheel.configurator.apply(leftCfg)

        val rightCfg = TalonFXConfiguration()
        rightCfg.MotorOutput = MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(InvertedValue.Clockwise_Positive)
        rightCfg.Slot0 = Slot0Configs()
            .withKS(ShooterConstants.FLYWHEEL_kS)
            .withKV(ShooterConstants.FLYWHEEL_kV)
            .withKA(ShooterConstants.FLYWHEEL_kA)
            .withKP(ShooterConstants.FLYWHEEL_kP)
            .withKI(ShooterConstants.FLYWHEEL_kI)
            .withKD(ShooterConstants.FLYWHEEL_kD)
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
        topFlywheelGoalRps = 0.0
        bottomFlywheelGoalRps = 0.0
        leftFlywheel.setControl(neutralRequest)
        rightFlywheel.setControl(neutralRequest)
    }

    fun homeTurret() = setTurretGoalRot(ShooterConstants.TURRET_HOME_ROT)
    fun homeHood() = setHoodGoalRot(ShooterConstants.HOOD_HOME_ROT)

    fun turretAtGoal(): Boolean =
        abs(turretMotor.position.valueAsDouble - turretGoalRot) < ShooterConstants.TURRET_TOLERANCE_ROT

    fun hoodAtGoal(): Boolean =
        abs(hoodMotor.position.valueAsDouble - hoodGoalRot) < ShooterConstants.HOOD_TOLERANCE_ROT

    fun flywheelsAtSpeed(): Boolean =
        abs(leftFlywheel.velocity.valueAsDouble - topFlywheelGoalRps) < ShooterConstants.FLYWHEEL_TOLERANCE_RPS &&
                abs(rightFlywheel.velocity.valueAsDouble - bottomFlywheelGoalRps) < ShooterConstants.FLYWHEEL_TOLERANCE_RPS

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
    }
}