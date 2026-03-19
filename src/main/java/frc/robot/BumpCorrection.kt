package frc.robot

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Rotation2d
import kotlin.math.abs
import kotlin.math.sign

class BumpCorrection {
    private var chosenTargetHeading = Rotation2d.fromDegrees(45.0)
    private var wasInBump = false
    private var entryHeading = Rotation2d.kZero
    private var entryProgress = 0.0

    fun getTargetHeading(
        zone: Int,
        currentHeading: Rotation2d,
        preferredYSign: Double
    ): Rotation2d {
        val inBump = BumpZones.isBumpZone(zone)

        if (inBump && !wasInBump) {
            entryHeading = currentHeading
            entryProgress = zoneProgress(zone)
            chosenTargetHeading = nearestDiagonal(currentHeading, preferredYSign)
        }

        wasInBump = inBump

        if (!inBump) { return currentHeading }

        val currentProgress = zoneProgress(zone)

        val totalToMid = abs(0.5 - entryProgress)
        val traveled = abs(currentProgress - entryProgress)

        val progress = if (totalToMid > 1e-6) {
            MathUtil.clamp(traveled / totalToMid, 0.0, 1.0)
        } else {
            1.0
        }

        return interpolateRotation(entryHeading, chosenTargetHeading, progress)
    }

    private fun nearestDiagonal(
        current: Rotation2d,
        preferredYSign: Double
    ): Rotation2d {
        val candidates = listOf(
            Rotation2d.fromDegrees(45.0),
            Rotation2d.fromDegrees(-45.0),
            Rotation2d.fromDegrees(135.0),
            Rotation2d.fromDegrees(-135.0)
        )

        val preferredSign = sign(preferredYSign)

        return candidates.minBy { candidate ->
            val angleError = abs(MathUtil.angleModulus(candidate.radians - current.radians))

            val candidateYSign = sign(candidate.sin)

            val tieBias = if (preferredSign != 0.0 && candidateYSign != preferredSign) {
                1e-3
            } else {
                0.0
            }

            angleError + tieBias
        }
    }

    private fun interpolateRotation(
        start: Rotation2d,
        end: Rotation2d,
        t: Double
    ): Rotation2d {
        val delta = MathUtil.angleModulus(end.radians - start.radians)
        return Rotation2d(start.radians + delta * t)
    }

    private fun zoneProgress(zone: Int): Double {
        return when (zone) {
            FieldZoneIds.BLUE_BUMP_TOP_LEFT,
            FieldZoneIds.BLUE_BUMP_BOTTOM_LEFT,
            FieldZoneIds.RED_BUMP_TOP_LEFT,
            FieldZoneIds.RED_BUMP_BOTTOM_LEFT -> 0.0

            FieldZoneIds.BLUE_BUMP_TOP_CENTER,
            FieldZoneIds.BLUE_BUMP_BOTTOM_CENTER,
            FieldZoneIds.RED_BUMP_TOP_CENTER,
            FieldZoneIds.RED_BUMP_BOTTOM_CENTER -> 0.5

            FieldZoneIds.BLUE_BUMP_TOP_RIGHT,
            FieldZoneIds.BLUE_BUMP_BOTTOM_RIGHT,
            FieldZoneIds.RED_BUMP_TOP_RIGHT,
            FieldZoneIds.RED_BUMP_BOTTOM_RIGHT -> 1.0

            else -> 0.5
        }
    }
}