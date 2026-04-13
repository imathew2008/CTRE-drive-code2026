package frc.robot.subsystems

import com.ctre.phoenix6.CANBus
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.TorqueCurrentFOC
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.MotorAlignmentValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsystems.IntakeConstants.ROLLER_VOLTAGE
import kotlin.math.abs
import kotlin.math.min


enum class IntakeState {
    IN,
    GOING_IN,
    STATIC,
    GOING_OUT,
    EXTENDED
}

enum class RollerState {
    IN,
    REVERSE,
    OFF
}

class IntakeSubsystem : SubsystemBase() {
    val kCANBus: CANBus = CANBus("Default Name", "./logs/example.hoot")
    private val deployMotor = TalonFX(IntakeConstants.EXTENSION_MOTOR_ID, kCANBus)
    private val rightRollerMotor = TalonFX(IntakeConstants.RIGHT_ROLLER_MOTOR_ID, kCANBus)
    private val lefRollerMotor = TalonFX(IntakeConstants.LEFT_ROLLER_MOTOR_ID, kCANBus)

    private val deployRequest = TorqueCurrentFOC(0.0)
    private val rollerRequest = VoltageOut(0.0)

    var currentHopperState: IntakeState = IntakeState.IN
        private set
    var desiredHopperState: IntakeState = IntakeState.IN
        private set
    var rollerState: RollerState = RollerState.OFF
        private set

    private val stateTimer = Timer()
    private var rampStartTorque = 0.0
    private var rampTargetTorque = 0.0
    private var rampingDown = false
    private var stallTimer = 0.0

    init {
        configureDeployMotor()
        configureRollerMotor()
        stateTimer.start()
    }

    //lots of fun request stuff
    fun requestExtended() {
        desiredHopperState = IntakeState.EXTENDED
    }

    fun requestIn() {
        desiredHopperState = IntakeState.IN
    }

    fun requestRunIntake() {
        rollerState = RollerState.IN
    }

    fun requestReverseIntake() {
        rollerState = RollerState.REVERSE
    }

    fun requestStopIntake() {
        rollerState = RollerState.OFF
    }

    fun requestToggle() {
        if(currentHopperState == IntakeState.EXTENDED || currentHopperState == IntakeState.GOING_OUT) {
            desiredHopperState = IntakeState.IN
        } else desiredHopperState = IntakeState.EXTENDED
    }

    override fun periodic() {
        updateStateMachine()
        extensionMotorOutputs()
        rollerMotorOutputs()
        publishTelemetry()
    }

    private fun updateStateMachine() {
        val t = stateTimer.get()

        when (currentHopperState) {
            IntakeState.IN -> if (desiredHopperState == IntakeState.EXTENDED || desiredHopperState == IntakeState.STATIC) {
                transitionTo(IntakeState.GOING_OUT)
            }

            IntakeState.EXTENDED -> if (desiredHopperState == IntakeState.IN || desiredHopperState == IntakeState.STATIC) {
                transitionTo(IntakeState.GOING_IN)
            }

            IntakeState.STATIC -> if (desiredHopperState == IntakeState.EXTENDED) {
                transitionTo(IntakeState.GOING_OUT)
            } else if (desiredHopperState == IntakeState.IN) {
                transitionTo(IntakeState.GOING_IN)
            }

            IntakeState.GOING_OUT -> if (desiredHopperState == IntakeState.IN || desiredHopperState == IntakeState.STATIC) {
                beginRampDown(IntakeConstants.STATIC_HOLD_TORQUE_A)
                if (rampDownComplete(t)) {
                    transitionTo(IntakeState.STATIC)
                }
            } else {
                updateRampUp(t, IntakeConstants.EXTEND_PEAK_TORQUE_A)
                if (detectStall(t)) {
                    transitionTo(IntakeState.EXTENDED)
                }
            }

            IntakeState.GOING_IN -> if (desiredHopperState == IntakeState.EXTENDED || desiredHopperState == IntakeState.STATIC) {
                beginRampDown(IntakeConstants.STATIC_HOLD_TORQUE_A)
                if (rampDownComplete(t)) {
                    transitionTo(IntakeState.STATIC)
                }
            } else {
                updateRampUp(t, IntakeConstants.RETRACT_PEAK_TORQUE_A)
                if (detectStall(t)) {
                    transitionTo(IntakeState.IN)
                }
            }
        }
    }

    private fun transitionTo(next: IntakeState) {
        currentHopperState = next
        rampingDown = false
        rampStartTorque = this.currentDeployTorque
        stallTimer = 0.0
        stateTimer.reset()

        if (next == IntakeState.EXTENDED && desiredHopperState == IntakeState.EXTENDED) {
            desiredHopperState = IntakeState.EXTENDED
        }
        if (next == IntakeState.IN && desiredHopperState == IntakeState.IN) {
            desiredHopperState = IntakeState.IN
        }
    }

    private fun updateRampUp(elapsedTime: Double, peakTorque: Double) {
        if (!rampingDown) {
            rampTargetTorque = peakTorque
        }
    }

    private fun beginRampDown(holdTorque: Double) {
        if (!rampingDown) {
            rampingDown = true
            rampStartTorque = this.currentDeployTorque
            rampTargetTorque = holdTorque
            stateTimer.reset()
        }
    }

    private fun rampDownComplete(elapsedTime: Double): Boolean {
        return rampingDown && elapsedTime >= IntakeConstants.RAMP_DOWN_S
    }

    private fun calculateDeployTorqueCommand(): Double {
        val t = stateTimer.get()

        when (currentHopperState) {
            IntakeState.GOING_OUT, IntakeState.GOING_IN -> {
                val rampPhase = if (rampingDown) IntakeConstants.RAMP_DOWN_S else IntakeConstants.RAMP_UP_S
                val alpha = min(t / rampPhase, 1.0)
                return lerp(rampStartTorque, rampTargetTorque, alpha)
            }

            IntakeState.EXTENDED ->
                return IntakeConstants.EXTEND_PEAK_TORQUE_A * 0.15

            IntakeState.IN ->
                return IntakeConstants.RETRACT_PEAK_TORQUE_A * 0.15

            IntakeState.STATIC -> return IntakeConstants.STATIC_HOLD_TORQUE_A
            else -> return 0.0
        }
    }

    private val currentDeployTorque: Double
        get() = deployMotor.closedLoopReference.valueAsDouble

    private fun detectStall(elapsedState: Double): Boolean {
        if (elapsedState < IntakeConstants.RAMP_UP_S) {
            stallTimer = 0.0
            return false
        }

        val current = abs(deployMotor.statorCurrent.valueAsDouble)
        if (current >= IntakeConstants.STALL_DETECT_A) {
            stallTimer += 0.02
        } else {
            stallTimer = 0.0
        }
        return stallTimer >= IntakeConstants.STALL_CONFIRM_S
    }

    private fun extensionMotorOutputs() {
        val deployCmd = calculateDeployTorqueCommand()
        deployMotor.setControl(deployRequest.withOutput(deployCmd))

        SmartDashboard.putNumber("Intake/DeployCmd_A", deployCmd)
        SmartDashboard.putNumber("Intake/DeployTorqueCurrent_A", deployMotor.torqueCurrent.valueAsDouble)
        SmartDashboard.putNumber("Intake/DeployStatorCurrent_A", deployMotor.statorCurrent.valueAsDouble)
    }

    private fun rollerMotorOutputs() {
        val rollerCmd = if (rollerState == RollerState.IN) ROLLER_VOLTAGE else 0.0
        rightRollerMotor.setControl(rollerRequest.withOutput(rollerCmd))
    }

    private fun configureDeployMotor() {
        val cfg = TalonFXConfiguration()

        cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake
        cfg.CurrentLimits.StatorCurrentLimit = 60.0
        cfg.CurrentLimits.StatorCurrentLimitEnable = true
        cfg.CurrentLimits.SupplyCurrentLimit = 40.0
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true

        deployMotor.configurator.apply(cfg)
    }

    private fun configureRollerMotor() {
        val cfg = TalonFXConfiguration()
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast
        rightRollerMotor.configurator.apply(cfg)
        lefRollerMotor.configurator.apply(cfg)
        lefRollerMotor.setControl(Follower(IntakeConstants.RIGHT_ROLLER_MOTOR_ID, MotorAlignmentValue.Opposed))
    }

    private fun publishTelemetry() {
        SmartDashboard.putString("Intake/CurrentState", currentHopperState.name)
        SmartDashboard.putString("Intake/DesiredState", desiredHopperState.name)
//        SmartDashboard.putNumber("Intake/DeployTorqueCurrent_A", deployMotor.torqueCurrent.valueAsDouble)
//        SmartDashboard.putNumber("Intake/DeployStatorCurrent_A", deployMotor.statorCurrent.valueAsDouble)
//        SmartDashboard.putNumber("Intake/DeployCmd_A", calculateDeployTorqueCommand())
        SmartDashboard.putNumber("Intake/StallTimer_s", stallTimer)
        SmartDashboard.putBoolean("Intake/IsRampingDown", rampingDown)
        SmartDashboard.putNumber("Intake/StateTimer_s", stateTimer.get())
    }

    private fun lerp(a: Double, b: Double, t: Double): Double {
        return a + (b - a) * t
    }
}