package frc.robot
import frc.robot.projectile.*
import kotlin.math.*

/**
 * Returns the angle of a vector in radians
 * @param v vector
 */
fun angle(v: Vector3) = atan2(v.y, v.x)
/**
 * Convert rpm to radians per second
 * @param rpm rpm to convert
 * @return radians per second
 */
fun rpmToRadPerSec(rpm: Double) = rpm * 2.0 * Math.PI / 60.0
/**
 * Converts radians per second to rpm
 * @param w radians per second to convert
 * @return rpm
 */
fun radPerSecToRpm(w: Double)   = w * 60.0 / (2.0 * Math.PI)
/**
 * Wraps angle(radians) to (-π, π]
 * @param a0 angle to wrap
 * @return angle between -π and π
 */
fun wrapAngle(a0: Double): Double {
    var a = a0
    while (a <= -Math.PI) a += 2.0 * Math.PI
    while (a > Math.PI) a -= 2.0 * Math.PI
    return a
}
/**
 * Computes wheel surface(tangential) speed in m/s from wheel diameter and RPM.
 * @param diameterM wheel diameter in meters
 * @param wheelRpm wheel speed in RPM
 * @return surface speed in m/s
 */
fun wheelSurfaceSpeed(diameterM: Double, wheelRpm: Double): Double {
    val r = diameterM * 0.5
    val w = rpmToRadPerSec(wheelRpm)
    return w * r
}
/**
 * Estimates the ball's exit linear velocity and spin from a two-roller shooter.
 *
 * Uses average wheel surface speed for exit velocity and wheel speed difference
 * to estimate ball angular velocity (spin) about the shooter spin axis.
 *
 * @param topWheelRadius radius of the top wheel (meters)
 * @param bottomWheelRadius radius of the bottom wheel (meters)
 * @param topRpm top wheel RPM
 * @param bottomRpm bottom wheel RPM
 * @param ballRadius ball radius (meters)
 * @param vScale empirical scaling factor for exit speed (defaults to 1.0)
 * @param spinScale empirical scaling factor for spin (defaults to 1.0)
 * @return BallExit containing exit speed (m/s), spin (rad/s), and spin in RPM
 */
fun ballExitFromTwoWheels(
    topWheelRadius:    Double,
    bottomWheelRadius: Double,
    topRpm:            Double,
    bottomRpm:         Double,
    ballRadius:        Double,
    vScale:            Double = 1.0,
    spinScale:         Double = 1.0
): BallExit {

    val uTop = wheelSurfaceSpeed(topWheelRadius, topRpm)
    val uBot = wheelSurfaceSpeed(bottomWheelRadius, bottomRpm)
    val vExit = vScale * 0.5 * (uTop + uBot)
    val omegaBall = spinScale * (uTop - uBot) / (2.0 * ballRadius)
    val ballRpm = radPerSecToRpm(omegaBall)

    return BallExit(vExit, omegaBall, ballRpm)
}
/**
 * Computes the Magnus (lift) force on a spinning projectile.
 *
 * Direction is given by ω × v. Magnitude is modeled using a lift coefficient
 * proportional to the spin ratio (R|ω|/|v|).
 *
 * Note: This assumes speed > 0 and ω × v is non-zero; callers should guard
 * against division by zero if needed.
 *
 * @param v projectile velocity vector (m/s)
 * @param omega projectile angular velocity vector (rad/s)
 * @return Magnus force vector (N)
 */
fun magnusForce(v: Vector3, omega: Vector3): Vector3 {
    val speed = v.norm
    val dir = omega.cross(v)
    val dirNorm = dir.norm
    val spinRatio = ballRadius * omega.norm / speed
    val cl = magnusCoeff * spinRatio
    val liftMag = 0.5 * airDensity * speed * speed * area * cl

    return (dir / dirNorm) * liftMag
}
/**
 * Computes aerodynamic drag force opposing the direction of motion.
 *
 * Uses a quadratic drag model: Fd = -0.5 * rho * Cd * A * |v| * v
 *
 * @param v projectile velocity vector (m/s)
 * @return drag force vector (N)
 */
fun dragForce(v: Vector3): Vector3 {
    val k = -0.5 * cd * airDensity * area
    return v * (k * v.norm)
}
/**
 * Computes the interpolation fraction 'a' where a segment crosses the plane x = xGoal.
 *
 * pCross = pPrev + a*(p - pPrev), where 0 <= a <= 1.
 *
 * @param pPrev previous position
 * @param p current position
 * @param xGoal x-plane to test against
 * @return interpolation fraction a in [0, 1], or null if no crossing / undefined
 */
fun crossXPlane(pPrev: Vector3, p: Vector3, xGoal: Double): Double? {
    val dx = p.x - pPrev.x
    if (dx == 0.0) return null
    val a = (xGoal - pPrev.x) / dx
    return if (a in 0.0..1.0) a else null
}
/**
 * Checks whether a candidate velocity is physically/algorithmically acceptable.
 *
 * Current policy:
 * - Must have positive x component (forward)
 * - Speed must be <= maxSpeed
 *
 * @param v candidate velocity vector
 * @return true if allowed, false otherwise
 */
fun possibleVelocity(v: Vector3): Boolean {
    if (v.x <= 0.0) return false
    if (v.norm > maxSpeed) return false
    return true
}

fun applyCircularDeadband(x: Double, y: Double, deadband: Double): DoubleArray {
    val mag = sqrt(x * x + y * y)
    if (mag < deadband) {
        return doubleArrayOf(0.0, 0.0)
    }
    val scaledMag = (mag - deadband) / (1.0 - deadband)
    val scale = scaledMag / mag

    return doubleArrayOf(
        x * scale,
        y * scale
    )
}

fun applyDeadband1D(value: Double, deadband: Double): Double {
    if (abs(value) <= deadband) {
        return 0.0
    }
    val scaled = (abs(value) - deadband) / (1.0 - deadband)
    return scaled.withSign(value)
}

fun squareKeepSign(x: Double): Double {
    return (x * x).withSign(x)
}

fun squareVectorKeepDirection(x: Double, y: Double): DoubleArray {
    val mag = hypot(x, y)
    if (mag <= 1e-9) return doubleArrayOf(0.0, 0.0)
    return doubleArrayOf(x * mag, y * mag)
}

fun getSpeed(speed: Double): Double {
    if(speed >= 8.0) {
        return 8.0
    }
    return speed
}

fun colorToZone(rgb: Int): Int {
    return when (rgb and 0xFFFFFF) {
        0x2F6FFF -> 1
        0x909090 -> 2
        0xFF4A4A -> 3
        0x000000 -> 0
        else -> 0
    }
}