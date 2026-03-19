package frc.robot

import edu.wpi.first.wpilibj.Filesystem
import org.w3c.dom.Element
import java.io.File
import javax.xml.parsers.DocumentBuilderFactory
import kotlin.math.hypot
import kotlin.math.roundToInt
import kotlin.math.sqrt

data class Vector2(var x: Double, var y: Double) {
    fun norm(): Double = hypot(x, y)
    fun plusAssign(o: Vector2) { x += o.x; y += o.y }
    fun times(s: Double): Vector2 = Vector2(x * s, y * s)
    fun minus(o: Vector2): Vector2 = Vector2(x - o.x, y - o.y)
}

/**
 * 3D vector used for physics calculations.
 *
 * Assumes a right-handed coordinate system:
 *  +x = forward
 *  +y = up
 *  +z = out of plane (lateral)
 *
 * All units are expected to be consistent (e.g., meters for position,
 * meters/second for velocity, Newtons for force).
 */
data class Vector3(val x: Double, val y: Double, val z: Double) {
    operator fun plus (v: Vector3) = Vector3(x + v.x, y + v.y, z + v.z)
    operator fun minus(v: Vector3) = Vector3(x - v.x, y - v.y, z - v.z)
    operator fun times(s: Double)  = Vector3(x * s, y * s, z * s)
    operator fun div  (s: Double)  = Vector3(x / s, y / s, z / s)

    val norm: Double get() = sqrt(x * x + y * y + z * z)

    fun cross(v: Vector3) = Vector3(
            y * v.z - z * v.y,
            z * v.x - x * v.z,
            x * v.y - y * v.x)

    companion object {
        val zero = Vector3(0.0, 0.0, 0.0)
    }
}

/**
 * Snapshot of projectile state at a simulation step.
 *
 * @property forces net force applied during the step (N)
 * @property pos position in world coordinates (m)
 * @property vel velocity in world coordinates (m/s)
 */
data class ResultsToPrint(val forces: Vector3, val pos: Vector3, val vel: Vector3)

/**
 * Represents the estimated exit conditions of a ball leaving
 * a two-flywheel shooter.
 *
 * @property vExit linear exit speed (m/s)
 * @property omegaBall angular velocity about spin axis (rad/s)
 * @property ballRpm angular velocity converted to RPM
 */
data class BallExit(
        val vExit:     Double,
        val omegaBall: Double,
        val ballRpm:   Double)

/**
 * Results from a full projectile simulation.
 *
 * Includes geometric target metrics, entry angle error,
 * optimization cost, and sampled trajectory data.
 *
 * @property hitWindow true if trajectory enters target window
 * @property crossedPlane true if trajectory crosses target x-plane
 * @property cost scalar optimization score(lower is better)
 * @property pCross interpolated position at plane crossing(if any)
 * @property vCross interpolated velocity at plane crossing(if any)
 * @property pBest closest position to target
 * @property vBest velocity at closest approach
 * @property results full list of sampled simulation states
 * @property above true if projectile passes above window
 * @property downward true if entry velocity is downward
 * @property phiError angular entry error relative to goal angle
 * @property distance closest spatial error to target
 */
data class SimResult(
        val hitWindow:    Boolean,
        val crossedPlane: Boolean,
        val cost:         Double,
        val pCross:       Vector3?,
        val vCross:       Vector3?,
        val pBest:        Vector3,
        val vBest:        Vector3,
        val results:      List<ResultsToPrint>,
        val above:        Boolean,
        val downward:     Boolean,
        val phiError:     Double,
        val distance:     Double
)

object FieldZoneIds {
    const val UNKNOWN = 0

    const val BLUE_ZONE    = 1
    const val NEUTRAL_ZONE = 2
    const val RED_ZONE     = 3

    const val BLUE_HUB_LEFT   = 10
    const val BLUE_HUB_CENTER = 11
    const val BLUE_HUB_RIGHT  = 12

    const val RED_HUB_LEFT   = 13
    const val RED_HUB_CENTER = 14
    const val RED_HUB_RIGHT  = 15

    const val BLUE_TRENCH_TOP_LEFT      = 20
    const val BLUE_TRENCH_TOP_CENTER    = 21
    const val BLUE_TRENCH_TOP_RIGHT     = 22
    const val BLUE_TRENCH_BOTTOM_LEFT   = 23
    const val BLUE_TRENCH_BOTTOM_CENTER = 24
    const val BLUE_TRENCH_BOTTOM_RIGHT  = 25

    const val BLUE_BUMP_TOP_LEFT   = 30
    const val BLUE_BUMP_TOP_CENTER = 31
    const val BLUE_BUMP_TOP_RIGHT  = 32

    const val BLUE_BUMP_BOTTOM_LEFT   = 33
    const val BLUE_BUMP_BOTTOM_CENTER = 34
    const val BLUE_BUMP_BOTTOM_RIGHT  = 35

    const val RED_BUMP_TOP_LEFT   = 36
    const val RED_BUMP_TOP_CENTER = 37
    const val RED_BUMP_TOP_RIGHT  = 38

    const val RED_BUMP_BOTTOM_LEFT   = 39
    const val RED_BUMP_BOTTOM_CENTER = 40
    const val RED_BUMP_BOTTOM_RIGHT  = 41

    const val RED_TRENCH_BOTTOM_LEFT   = 42
    const val RED_TRENCH_BOTTOM_CENTER  = 43
    const val RED_TRENCH_BOTTOM_RIGHT  = 44
    const val RED_TRENCH_TOP_LEFT   = 45
    const val RED_TRENCH_TOP_CENTER  = 46
    const val RED_TRENCH_TOP_RIGHT  = 47

    const val BLUE_OUTPOST = 50
    const val RED_OUTPOST  = 51

    const val BLUE_TOWER = 60
    const val RED_TOWER  = 61

    const val BLUE_DEPOT = 70
    const val RED_DEPOT  = 71
}

fun zoneName(zone: Int): String {
    return when (zone) {
        FieldZoneIds.UNKNOWN -> "UNKNOWN"

        FieldZoneIds.BLUE_ZONE -> "BLUE_ZONE"
        FieldZoneIds.NEUTRAL_ZONE -> "NEUTRAL_ZONE"
        FieldZoneIds.RED_ZONE -> "RED_ZONE"

        FieldZoneIds.BLUE_HUB_LEFT -> "BLUE_HUB_LEFT"
        FieldZoneIds.BLUE_HUB_CENTER -> "BLUE_HUB_CENTER"
        FieldZoneIds.BLUE_HUB_RIGHT -> "BLUE_HUB_RIGHT"

        FieldZoneIds.RED_HUB_LEFT -> "RED_HUB_LEFT"
        FieldZoneIds.RED_HUB_CENTER -> "RED_HUB_CENTER"
        FieldZoneIds.RED_HUB_RIGHT -> "RED_HUB_RIGHT"

        FieldZoneIds.BLUE_TRENCH_TOP_LEFT -> "BLUE_TRENCH_TOP_LEFT"
        FieldZoneIds.BLUE_TRENCH_TOP_CENTER -> "BLUE_TRENCH_TOP_CENTER"
        FieldZoneIds.BLUE_TRENCH_TOP_RIGHT -> "BLUE_TRENCH_TOP_RIGHT"

        FieldZoneIds.RED_TRENCH_TOP_LEFT -> "RED_TRENCH_TOP_LEFT"
        FieldZoneIds.RED_TRENCH_TOP_CENTER -> "RED_TRENCH_TOP_CENTER"
        FieldZoneIds.RED_TRENCH_TOP_RIGHT -> "RED_TRENCH_TOP_RIGHT"

        FieldZoneIds.BLUE_TRENCH_BOTTOM_LEFT -> "BLUE_TRENCH_BOTTOM_LEFT"
        FieldZoneIds.BLUE_TRENCH_BOTTOM_CENTER -> "BLUE_TRENCH_BOTTOM_CENTER"
        FieldZoneIds.BLUE_TRENCH_BOTTOM_RIGHT -> "BLUE_TRENCH_BOTTOM_RIGHT"

        FieldZoneIds.RED_TRENCH_BOTTOM_LEFT -> "RED_TRENCH_BOTTOM_LEFT"
        FieldZoneIds.RED_TRENCH_BOTTOM_CENTER -> "RED_TRENCH_BOTTOM_CENTER"
        FieldZoneIds.RED_TRENCH_BOTTOM_RIGHT -> "RED_TRENCH_BOTTOM_RIGHT"

        FieldZoneIds.BLUE_BUMP_TOP_LEFT -> "BLUE_BUMP_TOP_LEFT"
        FieldZoneIds.BLUE_BUMP_TOP_CENTER -> "BLUE_BUMP_TOP_CENTER"
        FieldZoneIds.BLUE_BUMP_TOP_RIGHT -> "BLUE_BUMP_TOP_RIGHT"

        FieldZoneIds.BLUE_BUMP_BOTTOM_LEFT -> "BLUE_BUMP_BOTTOM_LEFT"
        FieldZoneIds.BLUE_BUMP_BOTTOM_CENTER -> "BLUE_BUMP_BOTTOM_CENTER"
        FieldZoneIds.BLUE_BUMP_BOTTOM_RIGHT -> "BLUE_BUMP_BOTTOM_RIGHT"

        FieldZoneIds.RED_BUMP_TOP_LEFT -> "RED_BUMP_TOP_LEFT"
        FieldZoneIds.RED_BUMP_TOP_CENTER -> "RED_BUMP_TOP_CENTER"
        FieldZoneIds.RED_BUMP_TOP_RIGHT -> "RED_BUMP_TOP_RIGHT"

        FieldZoneIds.RED_BUMP_BOTTOM_LEFT -> "RED_BUMP_BOTTOM_LEFT"
        FieldZoneIds.RED_BUMP_BOTTOM_CENTER -> "RED_BUMP_BOTTOM_CENTER"
        FieldZoneIds.RED_BUMP_BOTTOM_RIGHT -> "RED_BUMP_BOTTOM_RIGHT"

        FieldZoneIds.BLUE_OUTPOST -> "BLUE_OUTPOST"
        FieldZoneIds.RED_OUTPOST -> "RED_OUTPOST"

        FieldZoneIds.BLUE_TOWER -> "BLUE_TOWER"
        FieldZoneIds.RED_TOWER -> "RED_TOWER"

        FieldZoneIds.BLUE_DEPOT -> "BLUE_DEPOT"
        FieldZoneIds.RED_DEPOT -> "RED_DEPOT"

        else -> "UNKNOWN($zone)"
    }
}

object BumpZones {
    @JvmStatic
    fun isBumpZone(zone: Int): Boolean {
        return when (zone) {
            FieldZoneIds.BLUE_BUMP_TOP_LEFT,
            FieldZoneIds.BLUE_BUMP_TOP_CENTER,
            FieldZoneIds.BLUE_BUMP_TOP_RIGHT,
            FieldZoneIds.BLUE_BUMP_BOTTOM_LEFT,
            FieldZoneIds.BLUE_BUMP_BOTTOM_CENTER,
            FieldZoneIds.BLUE_BUMP_BOTTOM_RIGHT,
            FieldZoneIds.RED_BUMP_TOP_LEFT,
            FieldZoneIds.RED_BUMP_TOP_CENTER,
            FieldZoneIds.RED_BUMP_TOP_RIGHT,
            FieldZoneIds.RED_BUMP_BOTTOM_LEFT,
            FieldZoneIds.RED_BUMP_BOTTOM_CENTER,
            FieldZoneIds.RED_BUMP_BOTTOM_RIGHT -> true

            else -> false
        }
    }
}

object TrenchZones {
    @JvmStatic
    fun isTrenchZone(zone: Int): Boolean {
        return when (zone) {
            FieldZoneIds.BLUE_TRENCH_TOP_LEFT,
            FieldZoneIds.BLUE_TRENCH_TOP_CENTER,
            FieldZoneIds.BLUE_TRENCH_TOP_RIGHT,
            FieldZoneIds.BLUE_TRENCH_BOTTOM_LEFT,
            FieldZoneIds.BLUE_TRENCH_BOTTOM_CENTER,
            FieldZoneIds.BLUE_TRENCH_BOTTOM_RIGHT,
            FieldZoneIds.RED_TRENCH_TOP_LEFT,
            FieldZoneIds.RED_TRENCH_TOP_CENTER,
            FieldZoneIds.RED_TRENCH_TOP_RIGHT,
            FieldZoneIds.RED_TRENCH_BOTTOM_LEFT,
            FieldZoneIds.RED_TRENCH_BOTTOM_CENTER,
            FieldZoneIds.RED_TRENCH_BOTTOM_RIGHT -> true

            else -> false
        }
    }
}

object FilteredFieldMap {
    const val WIDTH = 651
    const val HEIGHT = 318
    const val CELL_SIZE_INCHES = 1.0

    val ZONES: IntArray = loadZonesFromSvg()

    private fun loadZonesFromSvg(): IntArray {
        val zones = IntArray(WIDTH * HEIGHT) { FieldZoneIds.UNKNOWN }

        val file = File(Filesystem.getDeployDirectory(), "field_zones_filtered.svg")
        val doc = DocumentBuilderFactory.newInstance()
            .newDocumentBuilder()
            .parse(file)

        val rects = doc.getElementsByTagName("rect")

        for (i in 0 until rects.length) {
            val rect = rects.item(i) as? Element ?: continue

            val fill = rect.getAttribute("fill").lowercase()
            if (fill == "#111111") continue

            val x = rect.getAttribute("x").toDoubleOrNull() ?: continue
            val y = rect.getAttribute("y").toDoubleOrNull() ?: continue
            val w = rect.getAttribute("width").toDoubleOrNull() ?: continue
            val h = rect.getAttribute("height").toDoubleOrNull() ?: continue

            val zoneId = colorToZoneId(fill)
            val startX = (x / 2.0).roundToInt()
            val startY = (y / 2.0).roundToInt()
            val cellW = (w / 2.0).roundToInt().coerceAtLeast(1)
            val cellH = (h / 2.0).roundToInt().coerceAtLeast(1)

            for (dy in 0 until cellH) {
                for (dx in 0 until cellW) {
                    val gridX = startX + dx
                    val gridYFromTop = startY + dy

                    if (gridX !in 0 until WIDTH || gridYFromTop !in 0 until HEIGHT) continue

                    val gridY = HEIGHT - 1 - gridYFromTop
                    zones[gridY * WIDTH + gridX] = zoneId
                }
            }
        }

        return zones
    }
}


private fun colorToZoneId(fill: String): Int {
    return when (fill.lowercase()) {
        "#000000" -> FieldZoneIds.UNKNOWN

        "#2f6fff" -> FieldZoneIds.BLUE_ZONE
        "#909090" -> FieldZoneIds.NEUTRAL_ZONE
        "#ff4a4a" -> FieldZoneIds.RED_ZONE

        "#93b7ff" -> FieldZoneIds.BLUE_HUB_LEFT
        "#3c8dff" -> FieldZoneIds.BLUE_HUB_CENTER
        "#1d4ed8" -> FieldZoneIds.BLUE_HUB_RIGHT

        "#fca5a5" -> FieldZoneIds.RED_HUB_LEFT
        "#ef4444" -> FieldZoneIds.RED_HUB_CENTER
        "#b91c1c" -> FieldZoneIds.RED_HUB_RIGHT

        "#bcd3ff" -> FieldZoneIds.BLUE_TRENCH_TOP_LEFT
        "#60a5fa" -> FieldZoneIds.BLUE_TRENCH_TOP_CENTER
        "#2563eb" -> FieldZoneIds.BLUE_TRENCH_TOP_RIGHT

        "#fecaca" -> FieldZoneIds.RED_TRENCH_TOP_LEFT
        "#f87171" -> FieldZoneIds.RED_TRENCH_TOP_CENTER
        "#dc2626" -> FieldZoneIds.RED_TRENCH_TOP_RIGHT

        "#d1e1ff" -> FieldZoneIds.BLUE_TRENCH_BOTTOM_LEFT
        "#5b9df0" -> FieldZoneIds.BLUE_TRENCH_BOTTOM_CENTER
        "#235ede" -> FieldZoneIds.BLUE_TRENCH_BOTTOM_RIGHT

        "#ffcfcf" -> FieldZoneIds.RED_TRENCH_BOTTOM_LEFT
        "#ff7878" -> FieldZoneIds.RED_TRENCH_BOTTOM_CENTER
        "#d11b1b" -> FieldZoneIds.RED_TRENCH_BOTTOM_RIGHT

        "#c7d2fe" -> FieldZoneIds.BLUE_BUMP_TOP_LEFT
        "#818cf8" -> FieldZoneIds.BLUE_BUMP_TOP_CENTER
        "#4f46e5" -> FieldZoneIds.BLUE_BUMP_TOP_RIGHT

        "#a5b4fc" -> FieldZoneIds.BLUE_BUMP_BOTTOM_LEFT
        "#6366f1" -> FieldZoneIds.BLUE_BUMP_BOTTOM_CENTER
        "#4338ca" -> FieldZoneIds.BLUE_BUMP_BOTTOM_RIGHT

        "#fbcfe8" -> FieldZoneIds.RED_BUMP_TOP_LEFT
        "#ec4899" -> FieldZoneIds.RED_BUMP_TOP_CENTER
        "#be185d" -> FieldZoneIds.RED_BUMP_TOP_RIGHT

        "#f9a8d4" -> FieldZoneIds.RED_BUMP_BOTTOM_LEFT
        "#db2777" -> FieldZoneIds.RED_BUMP_BOTTOM_CENTER
        "#9d174d" -> FieldZoneIds.RED_BUMP_BOTTOM_RIGHT

        "#06b6d4" -> FieldZoneIds.BLUE_OUTPOST
        "#f97316" -> FieldZoneIds.RED_OUTPOST

        "#14b8a6" -> FieldZoneIds.BLUE_TOWER
        "#fb7185" -> FieldZoneIds.RED_TOWER

        "#22c55e" -> FieldZoneIds.BLUE_DEPOT
        "#eab308" -> FieldZoneIds.RED_DEPOT

        else -> FieldZoneIds.UNKNOWN
    }
}