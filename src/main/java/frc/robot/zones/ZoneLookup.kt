package frc.robot.zones

import edu.wpi.first.math.geometry.Pose2d

class ZoneLookup(
    private val width: Int,
    private val height: Int,
    private val cellSizeInches: Double,
    private val zones: IntArray
) {
    companion object {
        private const val INCHES_PER_METER = 39.3700787402
    }

    init {
        require(zones.size == width * height) {
            "Zone array size ${zones.size} does not match width*height = ${width * height}"
        }
    }

    fun getZoneAtPose(pose: Pose2d): Int {
        return getZoneAtMeters(pose.x, pose.y)
    }

    fun getZoneAtMeters(xMeters: Double, yMeters: Double): Int {
        val xInches = xMeters * INCHES_PER_METER
        val yInches = yMeters * INCHES_PER_METER

        val gridX = (xInches / cellSizeInches).toInt().coerceIn(0, width - 1)
        val gridY = (yInches / cellSizeInches).toInt().coerceIn(0, height - 1)

        return zones[gridY * width + gridX]
    }
}