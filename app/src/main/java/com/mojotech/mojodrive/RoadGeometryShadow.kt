package com.mojotech.mojodrive

import android.content.Context
import android.database.sqlite.SQLiteDatabase
import java.io.File
import kotlin.math.*

/**
 * Read-only road-geometry shadow evaluator.
 *
 * IMPORTANT: this class is diagnostic only. Its result is appended to field logs and is
 * deliberately NOT used by the proven v0.12 camera decision path.
 *
 * v0.14 policy changes are intentionally conservative:
 *  - multiple proven approaches for the same camera are evaluated together;
 *  - a matching/supporting approach always wins over a conflicting learned approach;
 *  - being far from a learned trace is OUTSIDE_COVERAGE, not a conflict;
 *  - conflict is emitted only when the vehicle is actually close to learned geometry and
 *    its heading strongly disagrees with every usable learned approach.
 *
 * This specifically prevents a partial learned trace from suppressing a real camera if the
 * shadow system is ever promoted in a later version. In v0.14 it still has zero engine effect.
 */
class RoadGeometryShadow(context: Context) {

    companion object {
        const val POLICY = "conservative_multi_approach_v2"
        private const val SUPPORT_DISTANCE_M = 70f
        private const val WEAK_SUPPORT_DISTANCE_M = 160f
        private const val COVERAGE_DISTANCE_M = 180f
        private const val SUPPORT_HEADING_DEG = 50f
        private const val WEAK_SUPPORT_HEADING_DEG = 65f
        private const val CONFLICT_HEADING_DEG = 105f
    }

    data class Result(
        val cameraId: String,
        val source: String,
        val confidence: String,
        val segmentId: Int,
        val segmentsConsidered: Int,
        val inCoverageSegments: Int,
        val corridorDistanceM: Float,
        val headingDeltaDeg: Float,
        val verdict: String
    ) {
        fun toLogFields(): String =
            "roadGeom=available;roadGeomCamera=$cameraId;roadGeomSource=$source;" +
                "roadGeomConfidence=$confidence;roadGeomPolicy=${RoadGeometryShadow.POLICY};" +
                "roadGeomSegment=$segmentId;roadGeomSegments=$segmentsConsidered;" +
                "roadGeomInCoverage=$inCoverageSegments;" +
                "roadGeomDistanceM=${fmt(corridorDistanceM)};" +
                "roadGeomHeadingDelta=${fmt(headingDeltaDeg)};roadGeomVerdict=$verdict;" +
                "roadGeomEngineEffect=false"

        companion object {
            private fun fmt(v: Float): String = String.format(java.util.Locale.US, "%.1f", v)
        }
    }

    private data class Point(val lat: Double, val lon: Double)

    private data class Segment(
        val id: Int,
        val cameraId: String,
        val cameraType: String,
        val source: String,
        val confidence: String,
        val points: List<Point>
    )

    private data class Candidate(
        val segment: Segment,
        val corridorDistanceM: Float,
        val headingDeltaDeg: Float,
        val verdict: String
    )

    private val segmentsByCamera = HashMap<String, MutableList<Segment>>()

    val segmentCount: Int
    val coveredCameraCount: Int

    init {
        val dbFile = File(context.filesDir, "road_geometry_v2.sqlite")
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
                        id = segmentId,
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

        val candidates = ArrayList<Candidate>()
        for (cameraId in cameraIds) {
            val segments = segmentsByCamera[key(cameraId, cameraType)] ?: continue
            for (segment in segments) {
                val nearest = nearestToPolyline(lat, lon, segment.points) ?: continue
                val headingDelta = if (vehicleBearingDeg != null) {
                    angleDifference(vehicleBearingDeg, nearest.second)
                } else {
                    180f
                }
                candidates += Candidate(
                    segment = segment,
                    corridorDistanceM = nearest.first,
                    headingDeltaDeg = headingDelta,
                    verdict = classify(nearest.first, headingDelta, vehicleBearingDeg != null)
                )
            }
        }

        if (candidates.isEmpty()) return null

        val total = candidates.size
        val inCoverage = candidates.count { it.corridorDistanceM <= COVERAGE_DISTANCE_M }

        // Conservative evidence ordering. A proven matching approach beats a different learned
        // approach that happens to be closer but points the wrong way. Conflict is selected only
        // when no support/weak-support/uncertain in-coverage alternative exists.
        val chosen = chooseBest(candidates, "support")
            ?: chooseBest(candidates, "weak_support")
            ?: chooseBest(candidates, "uncertain")
            ?: chooseBest(candidates, "position_only")
            ?: chooseBest(candidates, "conflict")
            ?: candidates.minByOrNull { it.corridorDistanceM }
            ?: return null

        return Result(
            cameraId = chosen.segment.cameraId,
            source = chosen.segment.source,
            confidence = chosen.segment.confidence,
            segmentId = chosen.segment.id,
            segmentsConsidered = total,
            inCoverageSegments = inCoverage,
            corridorDistanceM = chosen.corridorDistanceM,
            headingDeltaDeg = chosen.headingDeltaDeg,
            verdict = chosen.verdict
        )
    }

    private fun chooseBest(candidates: List<Candidate>, verdict: String): Candidate? {
        return candidates
            .asSequence()
            .filter { it.verdict == verdict }
            .minByOrNull {
                // Distance remains dominant; heading breaks ties between multiple valid approaches.
                it.corridorDistanceM + min(it.headingDeltaDeg, 180f) * 0.35f
            }
    }

    private fun classify(distanceM: Float, headingDeltaDeg: Float, bearingAvailable: Boolean): String {
        if (distanceM > COVERAGE_DISTANCE_M) return "outside_coverage"
        if (!bearingAvailable) return "position_only"
        if (distanceM <= SUPPORT_DISTANCE_M && headingDeltaDeg <= SUPPORT_HEADING_DEG) return "support"
        if (distanceM <= WEAK_SUPPORT_DISTANCE_M && headingDeltaDeg <= WEAK_SUPPORT_HEADING_DEG) return "weak_support"
        if (headingDeltaDeg >= CONFLICT_HEADING_DEG) return "conflict"
        return "uncertain"
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
