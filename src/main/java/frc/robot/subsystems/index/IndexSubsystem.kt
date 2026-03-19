package frc.robot.subsystems

import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage
import com.ctre.phoenix6.controls.NeutralOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsystems.IndexerConstants

class IndexSubsystem : SubsystemBase() {
    private val indexerMotor = TalonFX(IndexerConstants.INDEXER_MOTOR_ID)
    private val feederMotor = TalonFX(IndexerConstants.FEEDER_MOTOR_ID)

    private val indexerRequest = MotionMagicVelocityVoltage(0.0)
    private val feederRequest = MotionMagicVelocityVoltage(0.0)
    private val neutralRequest = NeutralOut()

    init {
        configIndexerMotor()
        configFeederMotor()
    }

    private fun configIndexerMotor() {
        val cfg = TalonFXConfiguration()
        cfg.MotorOutput = MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.CounterClockwise_Positive)

        cfg.Slot0 = Slot0Configs()
            .withKS(IndexerConstants.INDEXER_kS)
            .withKV(IndexerConstants.INDEXER_kV)
            .withKA(IndexerConstants.INDEXER_kA)
            .withKP(IndexerConstants.INDEXER_kP)
            .withKI(IndexerConstants.INDEXER_kI)
            .withKD(IndexerConstants.INDEXER_kD)

        cfg.MotionMagic.MotionMagicAcceleration = IndexerConstants.INDEXER_ACCEL_RPS_PER_SEC
        indexerMotor.configurator.apply(cfg)
    }

    private fun configFeederMotor() {
        val cfg = TalonFXConfiguration()
        cfg.MotorOutput = MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.CounterClockwise_Positive)

        cfg.Slot0 = Slot0Configs()
            .withKS(IndexerConstants.FEEDER_kS)
            .withKV(IndexerConstants.FEEDER_kV)
            .withKA(IndexerConstants.FEEDER_kA)
            .withKP(IndexerConstants.FEEDER_kP)
            .withKI(IndexerConstants.FEEDER_kI)
            .withKD(IndexerConstants.FEEDER_kD)

        cfg.MotionMagic.MotionMagicAcceleration = IndexerConstants.FEEDER_ACCEL_RPS_PER_SEC
        feederMotor.configurator.apply(cfg)
    }

    fun runIndexer() {
        indexerMotor.setControl(indexerRequest.withVelocity(IndexerConstants.INDEXER_TARGET_RPS))
    }

    fun stopIndexer() {
        indexerMotor.setControl(neutralRequest)
    }

    fun runIndexerReverse() {
        indexerMotor.setControl(
            indexerRequest.withVelocity(-IndexerConstants.INDEXER_TARGET_RPS)
        )
    }

    fun runFeeder() {
        feederMotor.setControl(feederRequest.withVelocity(IndexerConstants.FEEDER_TARGET_RPS))
    }

    fun stopFeeder() {
        feederMotor.setControl(neutralRequest)
    }

    fun runFeederReverse() {
        feederMotor.setControl(
            feederRequest.withVelocity(-IndexerConstants.FEEDER_TARGET_RPS)
        )
    }
}