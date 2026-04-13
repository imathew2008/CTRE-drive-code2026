package frc.robot.zones

import edu.wpi.first.math.geometry.Pose2d
import frc.robot.FieldZoneIds

class StableZoneLookup(private val zoneLookup: ZoneLookup) {
    private var lastKnownZone: Int = FieldZoneIds.UNKNOWN

    fun reset(zone: Int = FieldZoneIds.UNKNOWN) {
        lastKnownZone = zone
    }

    fun getStableZone(pose: Pose2d): Int {
        val zone = zoneLookup.getZoneAtPose(pose)
        if (zone != FieldZoneIds.UNKNOWN) {
            lastKnownZone = zone
        }
        return lastKnownZone
    }
}