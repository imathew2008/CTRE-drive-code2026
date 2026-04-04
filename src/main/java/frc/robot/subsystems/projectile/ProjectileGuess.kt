package frc.robot.subsystems.projectile

import kotlin.math.*

data class Vector3(val x: Double, val y: Double, val z: Double) {
    operator fun plus (v: Vector3) = Vector3(x + v.x, y + v.y, z + v.z)
    operator fun minus(v: Vector3) = Vector3(x - v.x, y - v.y, z - v.z)
    operator fun times(s: Double)  = Vector3(x * s, y * s, z * s)
    operator fun div  (s: Double)  = Vector3(x / s, y / s, z / s)
}

fun analyticInitialGuess(
    p0: Vector3,
    goal: Vector3 = Vector3(3.05, 1.8, 0.0),
    desiredImpactAngle: Double = Math.toRadians(-45.0),
    g: Double = 9.81
): Vector3? {
    val dx = goal.x - p0.x   
    val dz = goal.z - p0.z
    val dy = goal.y - p0.y

    val R = hypot(dx, dz)
    if (R < 1e-9) return null

    val phi = desiredImpactAngle
    val tanTheta = 2.0 * dy / R - tan(phi)
    val theta = atan(tanTheta)

    val denom = cos(theta).pow(2) * (tan(theta) - tan(phi))
    if (denom <= 1e-9) return null

    val v2 = g * R / denom
    if (v2 <= 0.0) return null

    val v = sqrt(v2)
    val yaw = atan2(dz, dx)

    return Vector3(
        v * cos(theta) * cos(yaw),
        v * sin(theta),
        v * cos(theta) * sin(yaw)
    )
}