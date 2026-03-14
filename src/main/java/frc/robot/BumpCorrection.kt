package frc.robot

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Rotation2d
import kotlin.math.abs

class BumpCorrection(
    private val bumpStartX: Double,
    private val bumpEndX: Double
) {
    private var chosenTargetHeading = Rotation2d.fromDegrees(45.0)
    private var wasInBump = false
    private var entryHeading = Rotation2d.kZero
    private var entryX = 0.0

    fun reset() {
        wasInBump = false
        entryHeading = Rotation2d.kZero
        entryX = 0.0
        chosenTargetHeading = Rotation2d.fromDegrees(45.0)
    }

    fun isInBump(x: Double): Boolean {
        return x in bumpStartX..bumpEndX
    }

    fun getTargetHeading(currentX: Double, currentHeading: Rotation2d): Rotation2d {
        val inBump = isInBump(currentX)
        val bumpMidX = (bumpStartX + bumpEndX) / 2.0

        if (inBump && !wasInBump) {
            entryHeading = currentHeading
            entryX = currentX
            chosenTargetHeading = nearestDiagonal(currentHeading)
        }

        wasInBump = inBump

        if (!inBump) {
            return currentHeading
        }

        val totalToMid = abs(bumpMidX - entryX)
        val traveled = abs(currentX - entryX)

        val progress = if (totalToMid > 1e-6) {
            MathUtil.clamp(traveled / totalToMid, 0.0, 1.0)
        } else {
            1.0
        }

        return interpolateRotation(entryHeading, chosenTargetHeading, progress)
    }

    fun getChosenTargetHeading(): Rotation2d {
        return chosenTargetHeading
    }

    private fun nearestDiagonal(current: Rotation2d): Rotation2d {
        val candidates = listOf(
            Rotation2d.fromDegrees(45.0),
            Rotation2d.fromDegrees(-45.0),
            Rotation2d.fromDegrees(135.0),
            Rotation2d.fromDegrees(-135.0)
        )

        return candidates.minBy { candidate ->
            abs(MathUtil.angleModulus(candidate.radians - current.radians))
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
}