package frc.robot.subsystems

object IntakeConstants {
    const val EXTENSION_MOTOR_ID    = 17
    const val LEFT_ROLLER_MOTOR_ID  = 18
    const val RIGHT_ROLLER_MOTOR_ID = 19

    const val EXTEND_PEAK_TORQUE_A = 20.0
    const val RETRACT_PEAK_TORQUE_A = -20.0
    const val STATIC_HOLD_TORQUE_A = 3.0
    const val ROLLER_VOLTAGE = 4.0
    const val RAMP_UP_S = 0.25
    const val RAMP_DOWN_S = 0.15
    const val STALL_DETECT_A = 25.0
    const val STALL_CONFIRM_S = 0.08
}

object IndexerConstants {
    const val INDEXER_MOTOR_ID = 60
    const val FEEDER_MOTOR_ID  = 61

    const val INDEXER_RUN_VOLTAGE = 4.5
    const val INDEXER_REVERSE_VOLTAGE  = -3.0

    const val TRIGGER_RUN_VOLTAGE = 5.0
    const val TRIGGER_REVERSE_VOLTAGE = -2.5
}

object ShooterConstants {
    const val HOOD_MOTOR_ID = 12
    const val TURRET_MOTOR_ID = 13
    const val LEFT_FLYWHEEL_MOTOR_ID = 14
    const val RIGHT_FLYWHEEL_MOTOR_ID = 15

    const val TURRET_ENCODER_ID = 10
    const val HOOD_ENCODER_ID = 11

    // -------------------------
    // Turret
    // -------------------------
    //1:8
    //8:1, 9:1
    const val TURRET_GEAR_RATIO = 8.0
    const val TURRET_SENSOR_TO_MECHANISM_RATIO = 0.888888889
    const val TURRET_ROTOR_TO_SENSOR_RATIO = 8.0 / TURRET_GEAR_RATIO

    @JvmField val TURRET_MIN_ROT = -0.25
    @JvmField val TURRET_MAX_ROT = 0.25
    @JvmField val TURRET_HOME_ROT = 0.0

    const val TURRET_kS = 0.0
    const val TURRET_kV = 0.0
    const val TURRET_kA = 0.0
    const val TURRET_kP = 40.0
    const val TURRET_kI = 0.0
    const val TURRET_kD = 0.0

    const val TURRET_CRUISE_VEL = 0.75
    const val TURRET_ACCEL = 1.5
    const val TURRET_JERK = 0.0

    const val TURRET_TOLERANCE_ROT = 0.01

    // -------------------------
    // Hood
    // -------------------------
    //10:19
    //15:38
    const val HOOD_GEAR_RATIO = 1.9
    const val HOOD_SENSOR_TO_MECHANISM_RATIO = 38.0 / 15.0
    const val HOOD_ROTOR_TO_SENSOR_RATIO = (19.0 / 10.0) / (38.0 / 15.0)

    @JvmField val HOOD_MIN_ROT  = 0.0
    @JvmField val HOOD_MAX_ROT  = 0.18
    @JvmField val HOOD_HOME_ROT = 0.02

    const val HOOD_kS = 0.0
    const val HOOD_kV = 0.0
    const val HOOD_kA = 0.0
    const val HOOD_kP = 50.0
    const val HOOD_kI = 0.0
    const val HOOD_kD = 0.0
    const val HOOD_kG = 0.0

    const val HOOD_CRUISE_VEL = 0.5
    const val HOOD_ACCEL = 1.0
    const val HOOD_JERK = 0.0

    const val HOOD_TOLERANCE_ROT = 0.005

    // -------------------------
    // Flywheels
    // -------------------------
    const val FLYWHEEL_kS = 0.0
    const val FLYWHEEL_kV = 0.12
    const val FLYWHEEL_kA = 0.0
    const val FLYWHEEL_kP = 0.15
    const val FLYWHEEL_kI = 0.0
    const val FLYWHEEL_kD = 0.0

    const val TOP_FLYWHEEL_TARGET_RPS = 70.0
    const val BOTTOM_FLYWHEEL_TARGET_RPS = 50.0
    const val FLYWHEEL_TOLERANCE_RPS = 3.0
}