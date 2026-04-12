package frc.robot.subsystems

import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage
import com.ctre.phoenix6.controls.NeutralOut
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.wpilibj2.command.SubsystemBase

enum class IndexState {
    IN,
    REVERSE,
    OFF
}

enum class TriggerState {
    IN,
    REVERSE,
    OFF
}

class IndexSubsystem : SubsystemBase() {
    private val indexMotor = TalonFX(IndexerConstants.INDEXER_MOTOR_ID)
    private val triggerMotor = TalonFX(IndexerConstants.FEEDER_MOTOR_ID)

    private val indexerRequest = VoltageOut(0.0)
    private val triggerRequest = VoltageOut(0.0)

    var currentIndexState: IndexState = IndexState.OFF
    private set
    var currentTriggerState: TriggerState = TriggerState.OFF
    private set

    init {
        configureIndexerMotor()
        configureTriggerMotor()
    }

    fun requestRunIndexer() {
        currentIndexState = IndexState.IN
    }

    fun requestReverseIndexer() {
        currentIndexState = IndexState.REVERSE
    }

    fun requestStopIndexer() {
        currentIndexState = IndexState.OFF
    }

    fun requestRunTrigger() {
        currentTriggerState = TriggerState.IN
    }

    fun requestReverseTrigger() {
        currentTriggerState = TriggerState.REVERSE
    }

    fun requestStopTrigger() {
        currentTriggerState = TriggerState.OFF
    }

    override fun periodic() {
        indexerMotorOutput()
        triggerMotorOutput()
    }

    private fun indexerMotorOutput() {
        val indexerCmd = when (currentIndexState) {
            IndexState.IN -> {
                IndexerConstants.INDEXER_RUN_VOLTAGE
            }
            IndexState.REVERSE -> {
                IndexerConstants.INDEXER_REVERSE_VOLTAGE
            }
            else -> 0.0
        }
        indexMotor.setControl(indexerRequest.withOutput(indexerCmd))
    }

    private fun triggerMotorOutput() {
        val triggerCmd = when (currentTriggerState) {
            TriggerState.IN -> {
                IndexerConstants.TRIGGER_RUN_VOLTAGE
            }
            TriggerState.REVERSE -> {
                IndexerConstants.TRIGGER_REVERSE_VOLTAGE
            }
            else -> 0.0
        }
        triggerMotor.setControl(triggerRequest.withOutput(triggerCmd))
    }

    private fun configureIndexerMotor() {
        val cfg = TalonFXConfiguration()
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast
        cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive
        indexMotor.configurator.apply(cfg)
    }

    private fun configureTriggerMotor() {
        val cfg = TalonFXConfiguration()
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast
        triggerMotor.configurator.apply(cfg)
    }
}