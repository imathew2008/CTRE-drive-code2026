package frc.robot.subsystems.intake

import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.TorqueCurrentFOC
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

    private val extensionRequest = TorqueCurrentFOC(0.0)
    private val leftRollerRequest = VoltageOut(0.0)
    private val rightRollerRequest = VoltageOut(0.0)

    private var goalPositionRot = IntakeConstants.STOW_EXTENSION_ROT
    private var extensionClosedLoopEnabled = true

    private var extensionAmps = 3.0
    private val extensionMaxAbsCurrent = 40.0
    var extended = false

    init {
        configExtensionMotor()
        configRollerMotors()
        extensionMotor.setPosition(IntakeConstants.STOW_EXTENSION_ROT)
        goalPositionRot = IntakeConstants.STOW_EXTENSION_ROT
        var extended = false
    }

    private fun configExtensionMotor() {
        val cfg = TalonFXConfiguration()

        cfg.MotorOutput = MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.CounterClockwise_Positive)

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
            .withInverted(InvertedValue.Clockwise_Positive)

        leftRollerMotor.configurator.apply(leftCfg)
        rightRollerMotor.configurator.apply(rightCfg)
    }

    fun zeroAtStow() {
        extensionMotor.setPosition(IntakeConstants.STOW_EXTENSION_ROT)
        goalPositionRot = IntakeConstants.STOW_EXTENSION_ROT
        extensionClosedLoopEnabled = true
    }

    fun getExtensionPositionRot(): Double {
        return extensionMotor.position.valueAsDouble
    }

    fun getExtensionVelocityRps(): Double {
        return extensionMotor.velocity.valueAsDouble
    }

    fun setGoalPosition(positionRot: Double) {
        goalPositionRot = MathUtil.clamp(
            positionRot,
            IntakeConstants.MIN_EXTENSION_ROT,
            IntakeConstants.MAX_EXTENSION_ROT
        )
        extensionClosedLoopEnabled = true
    }

    fun deploy() = setGoalPosition(IntakeConstants.INTAKE_EXTENSION_ROT)

    fun stow() = setGoalPosition(IntakeConstants.STOW_EXTENSION_ROT)

    fun atGoal(): Boolean {
        return abs(getExtensionPositionRot() - goalPositionRot) <
                IntakeConstants.POSITION_TOLERANCE_ROT
    }

    fun setExtensionCurrent(amps: Double) {
        extensionClosedLoopEnabled = false
        val clamped = MathUtil.clamp(amps, -extensionMaxAbsCurrent, extensionMaxAbsCurrent)
        extensionMotor.setControl(extensionRequest.withOutput(clamped))
    }

    fun stopExtension() {
        extensionClosedLoopEnabled = false
        extensionMotor.setControl(extensionRequest.withOutput(0.0))
    }

    fun setRollerVoltage(volts: Double) {
        leftRollerMotor.setControl(leftRollerRequest.withOutput(volts))
        rightRollerMotor.setControl(rightRollerRequest.withOutput(volts))
    }

    fun stopRollers() {
        leftRollerMotor.setControl(leftRollerRequest.withOutput(0.0))
        rightRollerMotor.setControl(rightRollerRequest.withOutput(0.0))
    }

    fun intakeActuation() {
        if(extended) {
            extensionAmps = -extensionAmps
            extended = false
        } else {
            extensionAmps = -extensionAmps
            extended = true
        }
    }

    fun retractIntake() {
        extensionMotor.setControl(extensionRequest.withOutput(-extensionAmps))
    }

    override fun periodic() {
//        val positionRot = getExtensionPositionRot()
//        val velocityRps = getExtensionVelocityRps()

        extensionMotor.setControl(extensionRequest.withOutput(extensionAmps))

//        if (extensionClosedLoopEnabled) {
//            val errorRot = goalPositionRot - positionRot
//
//            val commandedCurrent = extensionAmps
//
//            SmartDashboard.putNumber("Intake/ExtensionCmdCurrentAmps", commandedCurrent)
//        }
//
//        SmartDashboard.putNumber("Intake/GoalRot", goalPositionRot)
//        SmartDashboard.putNumber("Intake/PositionRot", positionRot)
//        SmartDashboard.putNumber("Intake/VelocityRps", velocityRps)
//        SmartDashboard.putBoolean("Intake/AtGoal", atGoal())
//        SmartDashboard.putBoolean("Intake/ExtensionClosedLoopEnabled", extensionClosedLoopEnabled)
    }
}