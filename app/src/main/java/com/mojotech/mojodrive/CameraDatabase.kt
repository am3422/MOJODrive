package com.mojotech.mojodrive

import android.content.Context
import android.database.sqlite.SQLiteDatabase
import java.io.File

data class CameraPoint(
    val id: String,
    val type: String,
    val latitude: Double,
    val longitude: Double,
    val speedLimit: Int?,
    val roadName: String,
    val roadHighway: String,
    val roadDistanceM: Double?,
    val speedSource: String,
    val speedConfidence: String,
    val roadBearing: Float?,
    val oneway: Boolean,
    val directionMode: String,
    val directionConfidence: String
)

class CameraDatabase(private val context: Context) {

    // Force a fresh copy of the bundled Neshan-derived database for 0.8.
    // This prevents an old copied DB from surviving an APK update.
    private val dbFile = File(context.filesDir, "shiraz_cameras_v08.sqlite")

    private fun ensureDatabase() {
        if (dbFile.exists() && dbFile.length() > 0L) return

        context.assets.open("shiraz_cameras.sqlite").use { input ->
            dbFile.outputStream().use { output ->
                input.copyTo(output)
            }
        }
    }

    fun loadAll(): List<CameraPoint> {
        ensureDatabase()

        val db = SQLiteDatabase.openDatabase(
            dbFile.absolutePath,
            null,
            SQLiteDatabase.OPEN_READONLY
        )

        val result = ArrayList<CameraPoint>()

        db.rawQuery(
            "SELECT camera_id, camera_type, latitude, longitude, speed_limit, road_name, road_highway, " +
                "road_distance_m, speed_source, speed_confidence, road_bearing, oneway, " +
                "direction_mode, direction_confidence FROM cameras",
            null
        ).use { c ->

            fun idx(name: String) = c.getColumnIndexOrThrow(name)

            while (c.moveToNext()) {
                result += CameraPoint(
                    id = c.getString(idx("camera_id")) ?: "",
                    type = c.getString(idx("camera_type")) ?: "speed",
                    latitude = c.getDouble(idx("latitude")),
                    longitude = c.getDouble(idx("longitude")),
                    speedLimit =
                        if (c.isNull(idx("speed_limit"))) null
                        else c.getInt(idx("speed_limit")),
                    roadName =
                        if (c.isNull(idx("road_name"))) ""
                        else c.getString(idx("road_name")),
                    roadHighway =
                        if (c.isNull(idx("road_highway"))) ""
                        else c.getString(idx("road_highway")),
                    roadDistanceM =
                        if (c.isNull(idx("road_distance_m"))) null
                        else c.getDouble(idx("road_distance_m")),
                    speedSource =
                        if (c.isNull(idx("speed_source"))) ""
                        else c.getString(idx("speed_source")),
                    speedConfidence =
                        if (c.isNull(idx("speed_confidence"))) "unknown"
                        else c.getString(idx("speed_confidence")),
                    roadBearing =
                        if (c.isNull(idx("road_bearing"))) null
                        else c.getFloat(idx("road_bearing")),
                    oneway =
                        !c.isNull(idx("oneway")) &&
                            c.getInt(idx("oneway")) == 1,
                    directionMode =
                        if (c.isNull(idx("direction_mode"))) "unknown"
                        else c.getString(idx("direction_mode")),
                    directionConfidence =
                        if (c.isNull(idx("direction_confidence"))) "unknown"
                        else c.getString(idx("direction_confidence"))
                )
            }
        }

        db.close()
        return result
    }
}
