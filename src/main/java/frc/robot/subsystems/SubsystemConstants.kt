package frc.robot.subsystems

import edu.wpi.first.networktables.NetworkTableInstance

object IntakeConstants {
    const val EXTENSION_MOTOR_ID    = 17
    const val LEFT_ROLLER_MOTOR_ID  = 18
    const val RIGHT_ROLLER_MOTOR_ID = 19

    const val EXTEND_PEAK_TORQUE_A = -9.5
    const val RETRACT_PEAK_TORQUE_A = 9.5
    const val STATIC_HOLD_TORQUE_A = 0.0
    const val ROLLER_VOLTAGE = -3.5
    const val RAMP_UP_S = 0.30
    const val RAMP_DOWN_S = 0.10
    const val STALL_DETECT_A = 11.0
    const val STALL_CONFIRM_S = 0.01
}

object IndexerConstants {
    const val INDEXER_MOTOR_ID = 60
    const val FEEDER_MOTOR_ID  = 61

    const val INDEXER_RUN_VOLTAGE = -4.5
    const val INDEXER_REVERSE_VOLTAGE  = 4.5

    const val TRIGGER_RUN_VOLTAGE = 5.0
    const val TRIGGER_REVERSE_VOLTAGE = -5.0
}

object ShooterConstants {
    private val nt = NetworkTableInstance.getDefault().getTable("Shooter")
    val ntFlywheelVelocity = nt.getDoubleTopic("Shooter/FlywheelVelocityMps").subscribe(0.0)
    val ntHoodAngleDeg     = nt.getDoubleTopic("Shooter/HoodAngleDeg").subscribe(0.0)
    val ntTurretAngleDeg   = nt.getDoubleTopic("Shooter/TurretAngleDeg").subscribe(0.0)

    const val HOOD_MOTOR_ID = 14
    const val TURRET_MOTOR_ID = 15
    const val TOP_FLYWHEEL_MOTOR_ID = 12
    const val BOTTOM_FLYWHEEL_MOTOR_ID = 13

    const val TURRET_ENCODER_ID = 11

    const val FLYWHEEL_ACCEL = 400.0

    const val UPPER_FLYWHEEL_DIAMETER_M = 2.0 * 0.0254  //2
    const val LOWER_FLYWHEEL_DIAMETER_M = 4.0 * 0.0254  //4
    const val FLYWHEEL_GEAR_RATIO       = 55.0 / 25.0    //55 : 25
    //5:11

    // -------------------------
    // Turret
    // -------------------------
    //32:124 = 8:31
    //68:13
    const val TURRET_PRIMARY_TO_MECHANISM_RATIO = (124.0 / 32.0) * (5.0 /1)
    const val TURRET_ABSOLUTE_TO_MECHANISM_RATIO = (210.0/44.0) * (68.0 /13.0)
//    const val TURRET_ABSOLUTE_TO_MECHANISM_RATIO = 68.0 / 13.0
//    const val TURRET_ROTOR_TO_SENSOR_RATIO = TURRET_GEAR_RATIO / TURRET_GEAR_RATIO

    @JvmField val TURRET_MIN_ROT = -0.25
    @JvmField val TURRET_MAX_ROT = 0.25
    @JvmField val TURRET_HOME_ROT = 0.0

    const val TURRET_kS = 0.0
    const val TURRET_kV = 0.0
    const val TURRET_kA = 0.0
    const val TURRET_kP = 0.0
    const val TURRET_kI = 0.0
    const val TURRET_kD = 0.0

    const val TURRET_CRUISE_VEL = 0.75
    const val TURRET_ACCEL = 1.5
    const val TURRET_JERK = 0.0

    const val TURRET_TOLERANCE_ROT = 0.01

    // -------------------------
    // Hood
    // -------------------------
    //16 : 27 : 15 : 164
    const val HOOD_GEAR_RATIO = 16.0 / 27.0 / 15.0 / 164.0

    const val HOOD_MIN_ROT  = 0.0
    const val HOOD_MAX_ROT  = 0.18
    const val HOOD_HOME_ROT = 0.02

    const val HOOD_kP = 0.0
    const val HOOD_kI = 0.0
    const val HOOD_kD = 0.0
    const val HOOD_kS = 0.0
    const val HOOD_kV = 0.0
    const val HOOD_kA = 0.0
    const val HOOD_kG = 0.0

    const val HOOD_CRUISE_VEL = 0.5
    const val HOOD_ACCEL = 1.0
    const val HOOD_JERK = 0.0

    const val HOOD_TOLERANCE_ROT = 0.005

    // -------------------------
    // Flywheels
    // -------------------------
    const val FLYWHEEL_kS = 0.0
    const val FLYWHEEL_kV = 0.0
    const val FLYWHEEL_kA = 0.0
    const val FLYWHEEL_kP = 0.0
    const val FLYWHEEL_kI = 0.0
    const val FLYWHEEL_kD = 0.0

    const val TOP_FLYWHEEL_TARGET_RPS    = 70.0
    const val BOTTOM_FLYWHEEL_TARGET_RPS = 50.0
    const val FLYWHEEL_TOLERANCE_RPS     = 3.0
}