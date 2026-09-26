package com.mojotech.mojodrive

import android.content.Context
import android.database.sqlite.SQLiteDatabase
import java.io.File
import kotlin.math.*

/**
 * Read-only road-geometry shadow evaluator.
 *
 * IMPORTANT: this class is diagnostic only. Its result is appended to field logs and is
 * deliberately NOT used by the v0.12 camera decision path. This lets us validate geometry
 * on real drives without risking a regression in the proven baseline engine.
 */
class RoadGeometryShadow(context: Context) {

    data class Result(
        val cameraId: String,
        val source: String,
        val confidence: String,
        val corridorDistanceM: Float,
        val headingDeltaDeg: Float,
        val verdict: String
    ) {
        fun toLogFields(): String =
            "roadGeom=available;roadGeomCamera=$cameraId;roadGeomSource=$source;" +
                "roadGeomConfidence=$confidence;roadGeomDistanceM=${fmt(corridorDistanceM)};" +
                "roadGeomHeadingDelta=${fmt(headingDeltaDeg)};roadGeomVerdict=$verdict"

        companion object {
            private fun fmt(v: Float): String = String.format(java.util.Locale.US, "%.1f", v)
        }
    }

    private data class Point(val lat: Double, val lon: Double)

    private data class Segment(
        val cameraId: String,
        val cameraType: String,
        val source: String,
        val confidence: String,
        val points: List<Point>
    )

    private val segmentsByCamera = HashMap<String, MutableList<Segment>>()

    val segmentCount: Int
    val coveredCameraCount: Int

    init {
        val dbFile = File(context.filesDir, "road_geometry_v1.sqlite")
        context.assets.open("road_geometry.sqlite").use { input ->
            dbFile.outputStream().use { output -> input.copyTo(output) }
        }

        val db = SQLiteDatabase.openDatabase(
            dbFile.absolutePath,
            null,
            SQLiteDatabase.OPEN_READONLY
        )

        try {
            val segmentRows = ArrayList<Triple<Int, Segment, MutableList<Point>>>()
            db.rawQuery(
                "SELECT segment_id,camera_id,camera_type,source_kind,confidence FROM road_segments ORDER BY segment_id",
                null
            ).use { c ->
                while (c.moveToNext()) {
                    val segmentId = c.getInt(0)
                    val pointList = ArrayList<Point>()
                    val segment = Segment(
                        cameraId = c.getString(1),
                        cameraType = c.getString(2),
                        source = c.getString(3),
                        confidence = c.getString(4),
                        points = pointList
                    )
                    segmentRows += Triple(segmentId, segment, pointList)
                }
            }

            val byId = segmentRows.associateBy { it.first }
            db.rawQuery(
                "SELECT segment_id,seq,latitude,longitude FROM road_points ORDER BY segment_id,seq",
                null
            ).use { c ->
                while (c.moveToNext()) {
                    val segmentId = c.getInt(0)
                    byId[segmentId]?.third?.add(Point(c.getDouble(2), c.getDouble(3)))
                }
            }

            for ((_, segment, _) in segmentRows) {
                if (segment.points.size < 2) continue
                val key = key(segment.cameraId, segment.cameraType)
                segmentsByCamera.getOrPut(key) { ArrayList() }.add(segment)
            }
        } finally {
            db.close()
        }

        segmentCount = segmentsByCamera.values.sumOf { it.size }
        coveredCameraCount = segmentsByCamera.keys.size
    }

    fun evaluate(
        cameraIds: List<String>,
        cameraType: String,
        lat: Double,
        lon: Double,
        vehicleBearingDeg: Float?
    ): Result? {
        if (!lat.isFinite() || !lon.isFinite()) return null

        var best: Result? = null
        for (cameraId in cameraIds) {
            val segments = segmentsByCamera[key(cameraId, cameraType)] ?: continue
            for (segment in segments) {
                val nearest = nearestToPolyline(lat, lon, segment.points) ?: continue
                val headingDelta = if (vehicleBearingDeg != null) {
                    angleDifference(vehicleBearingDeg, nearest.second)
                } else {
                    180f
                }

                val verdict = when {
                    vehicleBearingDeg == null -> "position_only"
                    nearest.first <= 55f && headingDelta <= 45f -> "support"
                    nearest.first <= 90f && headingDelta <= 60f -> "weak_support"
                    nearest.first > 140f || headingDelta > 100f -> "conflict"
                    else -> "uncertain"
                }

                val result = Result(
                    cameraId = cameraId,
                    source = segment.source,
                    confidence = segment.confidence,
                    corridorDistanceM = nearest.first,
                    headingDeltaDeg = headingDelta,
                    verdict = verdict
                )

                if (best == null || result.corridorDistanceM < best.corridorDistanceM) {
                    best = result
                }
            }
        }
        return best
    }

    private fun nearestToPolyline(
        lat: Double,
        lon: Double,
        points: List<Point>
    ): Pair<Float, Float>? {
        if (points.size < 2) return null

        val metersPerLon = 111_320.0 * cos(Math.toRadians(lat))
        val metersPerLat = 110_540.0

        var bestDistance = Double.POSITIVE_INFINITY
        var bestBearing = 0f

        for (i in 0 until points.lastIndex) {
            val a = points[i]
            val b = points[i + 1]

            // Current vehicle position is the local origin.
            val ax = (a.lon - lon) * metersPerLon
            val ay = (a.lat - lat) * metersPerLat
            val bx = (b.lon - lon) * metersPerLon
            val by = (b.lat - lat) * metersPerLat
            val vx = bx - ax
            val vy = by - ay
            val vv = vx * vx + vy * vy
            if (vv < 1e-6) continue

            val t = (-(ax * vx + ay * vy) / vv).coerceIn(0.0, 1.0)
            val qx = ax + t * vx
            val qy = ay + t * vy
            val distance = hypot(qx, qy)

            if (distance < bestDistance) {
                bestDistance = distance
                // Navigation bearing: 0 north, 90 east.
                bestBearing = normalizeBearing(Math.toDegrees(atan2(vx, vy)).toFloat())
            }
        }

        if (!bestDistance.isFinite()) return null
        return Pair(bestDistance.toFloat(), bestBearing)
    }

    private fun key(cameraId: String, cameraType: String): String = "$cameraType:$cameraId"

    private fun normalizeBearing(value: Float): Float {
        var x = value % 360f
        if (x < 0f) x += 360f
        return x
    }

    private fun angleDifference(a: Float, b: Float): Float {
        var d = abs(a - b) % 360f
        if (d > 180f) d = 360f - d
        return d
    }
}
