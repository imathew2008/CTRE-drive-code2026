package frc.robot.subsystems.intake

import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsystems.IntakeConstants
import kotlin.math.abs

class IntakeSubsystem : SubsystemBase() {
    private val extensionMotor = TalonFX(IntakeConstants.EXTENSION_MOTOR_ID)
    private val leftRollerMotor = TalonFX(IntakeConstants.LEFT_ROLLER_MOTOR_ID)
    private val rightRollerMotor = TalonFX(IntakeConstants.RIGHT_ROLLER_MOTOR_ID)

    private val motionMagicRequest = MotionMagicVoltage(0.0).withSlot(0)
    private val leftRollerRequest = VoltageOut(0.0)
    private val rightRollerRequest = VoltageOut(0.0)

    private var goalPositionRot = IntakeConstants.STOW_EXTENSION_ROT

    init {
        configExtensionMotor()
        configRollerMotors()

        // Only keep this if robot boots with intake fully stowed
        extensionMotor.setPosition(IntakeConstants.STOW_EXTENSION_ROT)
        goalPositionRot = IntakeConstants.STOW_EXTENSION_ROT
        extensionMotor.setControl(motionMagicRequest.withPosition(goalPositionRot))
    }

    private fun configExtensionMotor() {
        val cfg = TalonFXConfiguration()

        cfg.MotorOutput = MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.CounterClockwise_Positive)

        cfg.Slot0 = Slot0Configs()
            .withKS(IntakeConstants.kS)
            .withKV(IntakeConstants.kV)
            .withKA(IntakeConstants.kA)
            .withKP(IntakeConstants.kP)
            .withKI(IntakeConstants.kI)
            .withKD(IntakeConstants.kD)

        cfg.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.CRUISE_VELOCITY_RPS
        cfg.MotionMagic.MotionMagicAcceleration = IntakeConstants.ACCELERATION_RPS_PER_SEC
        cfg.MotionMagic.MotionMagicJerk = IntakeConstants.JERK_RPS_PER_SEC2

        extensionMotor.configurator.apply(cfg)
    }

    private fun configRollerMotors() {
        val leftCfg = TalonFXConfiguration()
        leftCfg.MotorOutput = MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.CounterClockwise_Positive)

        val rightCfg = TalonFXConfiguration()
        rightCfg.MotorOutput = MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            // flip this if the two rollers fight each other
            .withInverted(InvertedValue.Clockwise_Positive)

        leftRollerMotor.configurator.apply(leftCfg)
        rightRollerMotor.configurator.apply(rightCfg)
    }

    fun zeroAtStow() {
        extensionMotor.setPosition(IntakeConstants.STOW_EXTENSION_ROT)
        goalPositionRot = IntakeConstants.STOW_EXTENSION_ROT
        extensionMotor.setControl(motionMagicRequest.withPosition(goalPositionRot))
    }

    fun getExtensionPositionRot(): Double {
        return extensionMotor.position.valueAsDouble
    }

    fun setGoalPosition(positionRot: Double) {
        goalPositionRot = MathUtil.clamp(
            positionRot,
            IntakeConstants.MIN_EXTENSION_ROT,
            IntakeConstants.MAX_EXTENSION_ROT
        )
        extensionMotor.setControl(
            motionMagicRequest.withPosition(goalPositionRot)
        )
    }

    fun deploy() = setGoalPosition(IntakeConstants.INTAKE_EXTENSION_ROT)

    fun stow() = setGoalPosition(IntakeConstants.STOW_EXTENSION_ROT)

    fun atGoal(): Boolean {
        return abs(getExtensionPositionRot() - goalPositionRot) <
                IntakeConstants.POSITION_TOLERANCE_ROT
    }

    fun setRollerVoltage(volts: Double) {
        leftRollerMotor.setControl(leftRollerRequest.withOutput(volts))
        rightRollerMotor.setControl(rightRollerRequest.withOutput(volts))
    }

    fun stopRollers() {
        leftRollerMotor.setControl(leftRollerRequest.withOutput(0.0))
        rightRollerMotor.setControl(rightRollerRequest.withOutput(0.0))
    }

    override fun periodic() {
        SmartDashboard.putNumber("Intake/GoalRot", goalPositionRot)
        SmartDashboard.putNumber("Intake/PositionRot", getExtensionPositionRot())
        SmartDashboard.putBoolean("Intake/AtGoal", atGoal())
    }
}