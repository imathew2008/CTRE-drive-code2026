package frc.robot.subsystems.questNav

import frc.robot.Vector2
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sin
import kotlin.math.sqrt
import kotlin.random.Random

class GyroPointKinematic(private val robotSpeed: Double) {
    private val rng: Random = Random.Default
    private val p                   = Vector2(0.0, 0.0)
    private var lastRandomVector    = Vector2(0.0, 0.0)
    private var lastCorrectedVector = Vector2(0.0, 0.0)
    private val radiusMeters        = 0.01
    private var pendingStepFraction = 0.0
    private var pendingRandomAngle  = 0.0
    private var hasPendingStep = false

    init {
        randomizePoint()
        generatePendingStep()
    }

    fun randomizePoint() {
        val uMag = rng.nextDouble(0.0, 2.0 * Math.PI)
        val uAng = rng.nextDouble(0.0, 2.0 * Math.PI)

        val pointMagnitude = abs(sin(uMag)) * radiusMeters

        p.x = pointMagnitude * cos(uAng)
        p.y = pointMagnitude * sin(uAng)

        lastRandomVector = Vector2(0.0, 0.0)
        lastCorrectedVector = Vector2(0.0, 0.0)
    }

    private fun generatePendingStep() {
        pendingStepFraction = rng.nextDouble(0.0, 1.0)
        pendingRandomAngle = rng.nextDouble(0.0, 2.0 * Math.PI)
        hasPendingStep = true
    }

    private fun ensurePendingStep() {
        if (!hasPendingStep) {
            generatePendingStep()
        }
    }

    private fun adjustedDt(robotSpeedMetersPerSec: Double, baseDt: Double): Double {
        val speedRatio =
            (robotSpeedMetersPerSec / robotSpeed).coerceIn(0.0, 1.0)

        // higher speed -> smaller dt
        return baseDt / (1.0 + speedRatio)
    }

    private fun distanceToEdgeAlongAngle(angle: Double): Double {
        val ux = cos(angle)
        val uy = sin(angle)

        val px = p.x
        val py = p.y

        val dot = px * ux + py * uy
        val r2 = px * px + py * py
        val R2 = radiusMeters * radiusMeters

        val discriminant = dot * dot + (R2 - r2)
        if (discriminant <= 0.0) return 0.0

        return -dot + sqrt(discriminant)
    }

    fun update(robotSpeedMetersPerSec: Double, dt: Double) {
        ensurePendingStep()

        val speedRatio =
            (robotSpeedMetersPerSec / robotSpeed).coerceIn(0.0, 1.0)

        val dtAdjusted = adjustedDt(robotSpeedMetersPerSec, dt)

        val randomAngle = pendingRandomAngle
        val r = p.norm()

        // Blue magnitude: random fraction of available distance in that direction,
        // scaled by speed and speed-based dt.
        val distanceToEdge = distanceToEdgeAlongAngle(randomAngle)
        val rawStep = pendingStepFraction * distanceToEdge * speedRatio * (dtAdjusted / dt)

        if (rawStep == 0.0 || !rawStep.isFinite()) {
            lastRandomVector = Vector2(0.0, 0.0)
            lastCorrectedVector = Vector2(0.0, 0.0)
            generatePendingStep()
            return
        }

        val centerAngle =
            if (r < 1e-9) randomAngle
            else atan2(-p.y, -p.x)

        val distanceRatio = (r / radiusMeters).coerceIn(0.0, 1.0)

        // very strong near the boundary
        val correctionStrength = (distanceRatio.pow(2.0)).coerceIn(0.0, 1.0)

        val correctedAngle = blendAngles(randomAngle, centerAngle, correctionStrength)

        lastRandomVector = Vector2(
            rawStep * cos(randomAngle),
            rawStep * sin(randomAngle)
        )

        lastCorrectedVector = Vector2(
            rawStep * cos(correctedAngle),
            rawStep * sin(correctedAngle)
        )

        p.x += lastCorrectedVector.x
        p.y += lastCorrectedVector.y

        generatePendingStep()
    }

    fun position(): Vector2 = Vector2(p.x, p.y)

    private fun angleDiff(target: Double, source: Double): Double {
        var diff = target - source
        while (diff > Math.PI) diff -= 2.0 * Math.PI
        while (diff < -Math.PI) diff += 2.0 * Math.PI
        return diff
    }

    private fun blendAngles(from: Double, to: Double, t: Double): Double {
        val diff = angleDiff(to, from)
        return from + diff * t
    }
}