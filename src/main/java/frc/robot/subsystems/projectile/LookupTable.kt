package frc.robot.util

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import java.io.File
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.sqrt

object ShotLookupTables {
    init {

        SmartDashboard.putNumber("a/rps_a", 1.527)
        SmartDashboard.putNumber("a/rps_c", -4.122)
        SmartDashboard.putNumber("a/init_theta", 82.9)
    }

    data class DistanceVelocityRow(
        val distanceFt: Double,
        val vx: Double,
        val vy: Double
    )

    data class ShooterRow(
        val velocityFtS: Double,
        val topRps: Double,
        val bottomRps: Double,
        val hoodDeg: Double
    )

    data class ShotSolution(
        val rps: Double,
        val hoodDeg: Double
    )

    private var distanceToVelocityTable: List<DistanceVelocityRow> = emptyList()
    private var velocityToShooterTable: List<ShooterRow> = emptyList()

    @JvmStatic
    fun load() {
        val distanceToVelocityFile = "table1_distance_to_velocity.csv"
        val velocityToShooterFile = "table2_velocity_to_shooter.csv"
        distanceToVelocityTable = loadDistanceToVelocity(distanceToVelocityFile)
        velocityToShooterTable = loadVelocityToShooter(velocityToShooterFile)

        println("ShotLookupTables: loaded ${distanceToVelocityTable.size} distance rows")
        println("ShotLookupTables: loaded ${velocityToShooterTable.size} shooter rows")
    }

    fun isLoaded(): Boolean {
        return distanceToVelocityTable.isNotEmpty() && velocityToShooterTable.isNotEmpty()
    }

    fun getInterpolatedVelocity(distanceFt: Double): DistanceVelocityRow? {
        if (distanceToVelocityTable.isEmpty()) return null
        return interpolateDistanceToVelocity(distanceFt, distanceToVelocityTable)
    }

//    fun getInterpolatedShooter(velocityFtS: Pose2d): ShooterRow? {
//        if (velocityToShooterTable.isEmpty()) return null
//        return interpolateVelocityToShooter(velocityFtS, velocityToShooterTable)
//    }

    fun getShotForDistance(distance: Double): ShotSolution? {
        val velocity = getInterpolatedVelocity(distance * 3.2808399) ?: return null
        val vel = Translation2d(velocity.vx, velocity.vy)
        return ShotSolution(
            rps     = SmartDashboard.getNumber("a/rps_a", 1.527) * vel.norm + SmartDashboard.getNumber("a/rps_c", -4.122),
            hoodDeg = SmartDashboard.getNumber("a/init_theta", 82.9) - vel.angle.degrees
        )
    }

    private fun loadDistanceToVelocity(filename: String): List<DistanceVelocityRow> {
        val file = File(Filesystem.getDeployDirectory(), filename)

        if (!file.exists()) {
            println("ShotLookupTables: missing file ${file.absolutePath}")
            return emptyList()
        }

        return file.readLines()
            .drop(1)
            .mapNotNull { parseDistanceVelocityRow(it) }
            .sortedBy { it.distanceFt }
    }

    private fun loadVelocityToShooter(filename: String): List<ShooterRow> {
        val file = File(Filesystem.getDeployDirectory(), filename)

        if (!file.exists()) {
            println("ShotLookupTables: missing file ${file.absolutePath}")
            return emptyList()
        }

        return file.readLines()
            .drop(1)
            .mapNotNull { parseShooterRow(it) }
            .sortedBy { it.velocityFtS }
    }

    private fun parseDistanceVelocityRow(line: String): DistanceVelocityRow? {
        val parts = line.split(",").map { it.trim() }

        if (parts.size < 2) {
            println("ShotLookupTables: bad distance row: $line")
            return null
        }

        return try {
            DistanceVelocityRow(
                distanceFt = parts[0].toDouble(),
                vx = parts[1].toDouble(),
                vy = parts[2].toDouble()
            )
        } catch (e: Exception) {
            println("ShotLookupTables: failed distance row: $line")
            null
        }
    }

    private fun parseShooterRow(line: String): ShooterRow? {
        val parts = line.split(",").map { it.trim() }

        if (parts.size < 4) {
            println("ShotLookupTables: bad shooter row: $line")
            return null
        }

        return try {
            ShooterRow(
                velocityFtS = parts[0].toDouble(),
                topRps = parts[1].toDouble(),
                bottomRps = parts[2].toDouble(),
                hoodDeg = parts[3].toDouble()
            )
        } catch (e: Exception) {
            println("ShotLookupTables: failed shooter row: $line")
            null
        }
    }

    private fun interpolateDistanceToVelocity(
        distanceFt: Double,
        table: List<DistanceVelocityRow>
    ): DistanceVelocityRow {
        if (table.size == 1) return table.first()

        if (distanceFt <= table.first().distanceFt) return table.first()
        if (distanceFt >= table.last().distanceFt) return table.last()

        for (i in 0 until table.size - 1) {
            val a = table[i]
            val b = table[i + 1]

            if (distanceFt >= a.distanceFt && distanceFt <= b.distanceFt) {
                val t = (distanceFt - a.distanceFt) / (b.distanceFt - a.distanceFt)
                return DistanceVelocityRow(
                    distanceFt = distanceFt,
                    vx = lerp(a.vx, b.vx, t),
                    vy = lerp(a.vy, b.vy, t),
                )
            }
        }

        return table.last()
    }

//    private fun interpolateVelocityToShooter(
//        velocityFtS: Pose2d,
//        table: List<ShooterRow>
//    ): ShooterRow {
//        if (table.size == 1) return table.first()
//
//        if (velocityFtS <= table.first().velocityFtS) return table.first()
//        if (velocityFtS >= table.last().velocityFtS) return table.last()
//
//        for (i in 0 until table.size - 1) {
//            val a = table[i]
//            val b = table[i + 1]
//
//            if (velocityFtS >= a.velocityFtS && velocityFtS <= b.velocityFtS) {
//                val t = (velocityFtS - a.velocityFtS) / (b.velocityFtS - a.velocityFtS)
//
//                return ShooterRow(
//                    velocityFtS = velocityFtS,
//                    topRps = lerp(a.topRps, b.topRps, t),
//                    bottomRps = lerp(a.bottomRps, b.bottomRps, t),
//                    hoodDeg = lerp(a.hoodDeg, b.hoodDeg, t)
//                )
//            }
//        }
//
//        return table.last()
//    }

    private fun lerp(a: Double, b: Double, t: Double): Double {
        return a + (b - a) * t
    }
}