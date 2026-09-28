package com.mojotech.mojodrive

import android.content.Context
import android.database.sqlite.SQLiteDatabase
import java.io.File
import kotlin.math.*

/**
 * Conservative road-geometry evidence evaluator.
 *
 * Positive/generic road geometry remains diagnostic-only and cannot change the proven
 * v0.12 camera matcher. v0.15 adds one deliberately narrow live influence: an explicit,
 * user-confirmed NEGATIVE route may veto the FIRST alert for that camera when position
 * and heading closely match the labeled false-warning route.
 *
 * v0.15 adds a second evidence type: explicit field-confirmed NEGATIVE routes. These are
 * routes where the user actually drove while a camera warning was proven irrelevant (for
 * example, the camera is on the opposite carriageway/level). Negative evidence is never
 * inferred from a normal miss or from distance alone; it must be explicitly labeled in the
 * geometry DB. This keeps the learning conservative and prevents self-reinforcing mistakes.
 */
class RoadGeometryShadow(context: Context) {

    companion object {
        const val POLICY = "conservative_multi_approach_negative_route_v3"

        // Positive learned-road evidence (unchanged from v0.14).
        private const val SUPPORT_DISTANCE_M = 70f
        private const val WEAK_SUPPORT_DISTANCE_M = 160f
        private const val COVERAGE_DISTANCE_M = 180f
        private const val SUPPORT_HEADING_DEG = 50f
        private const val WEAK_SUPPORT_HEADING_DEG = 65f
        private const val CONFLICT_HEADING_DEG = 105f

        // Explicit negative-route evidence. It is deliberately narrow: the vehicle must be
        // close to a field-confirmed false-warning route and travelling along that route.
        private const val NEGATIVE_ROUTE_DISTANCE_M = 85f
        private const val NEGATIVE_ROUTE_HEADING_DEG = 60f
    }

    data class Result(
        val cameraId: String,
        val source: String,
        val confidence: String,
        val segmentId: Int,
        val segmentsConsidered: Int,
        val inCoverageSegments: Int,
        val negativeSegmentsConsidered: Int,
        val corridorDistanceM: Float,
        val headingDeltaDeg: Float,
        val verdict: String,
        val evidenceKind: String,
        val negativeLabel: String? = null
    ) {
        fun toLogFields(): String {
            val negative = if (negativeLabel.isNullOrBlank()) "" else ";roadGeomNegativeLabel=$negativeLabel"
            return "roadGeom=available;roadGeomCamera=$cameraId;roadGeomSource=$source;" +
                "roadGeomConfidence=$confidence;roadGeomPolicy=${RoadGeometryShadow.POLICY};" +
                "roadGeomEvidence=$evidenceKind;roadGeomSegment=$segmentId;" +
                "roadGeomSegments=$segmentsConsidered;roadGeomInCoverage=$inCoverageSegments;" +
                "roadGeomNegativeSegments=$negativeSegmentsConsidered;" +
                "roadGeomDistanceM=${fmt(corridorDistanceM)};" +
                "roadGeomHeadingDelta=${fmt(headingDeltaDeg)};roadGeomVerdict=$verdict;" +
                "roadGeomGenericEngineEffect=false;roadGeomExplicitNegativeVeto=true$negative"
        }

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

    private data class NegativeSegment(
        val id: Int,
        val cameraId: String,
        val cameraType: String,
        val source: String,
        val confidence: String,
        val label: String,
        val points: List<Point>
    )

    private data class Candidate(
        val segment: Segment,
        val corridorDistanceM: Float,
        val headingDeltaDeg: Float,
        val verdict: String
    )

    private data class NegativeCandidate(
        val segment: NegativeSegment,
        val corridorDistanceM: Float,
        val headingDeltaDeg: Float
    )

    private val segmentsByCamera = HashMap<String, MutableList<Segment>>()
    private val negativeSegmentsByCamera = HashMap<String, MutableList<NegativeSegment>>()

    val segmentCount: Int
    val positiveSegmentCount: Int
    val negativeSegmentCount: Int
    val coveredCameraCount: Int
    val negativeCoveredCameraCount: Int

    init {
        val dbFile = File(context.filesDir, "road_geometry_v3.sqlite")
        context.assets.open("road_geometry.sqlite").use { input ->
            dbFile.outputStream().use { output -> input.copyTo(output) }
        }

        val db = SQLiteDatabase.openDatabase(
            dbFile.absolutePath,
            null,
            SQLiteDatabase.OPEN_READONLY
        )

        try {
            loadPositiveSegments(db)
            // Backward-compatible: a v2 DB simply has no explicit negative evidence.
            try {
                loadNegativeSegments(db)
            } catch (_: Exception) {
                negativeSegmentsByCamera.clear()
            }
        } finally {
            db.close()
        }

        positiveSegmentCount = segmentsByCamera.values.sumOf { it.size }
        negativeSegmentCount = negativeSegmentsByCamera.values.sumOf { it.size }
        segmentCount = positiveSegmentCount + negativeSegmentCount
        coveredCameraCount = (segmentsByCamera.keys + negativeSegmentsByCamera.keys).toSet().size
        negativeCoveredCameraCount = negativeSegmentsByCamera.keys.size
    }

    private fun loadPositiveSegments(db: SQLiteDatabase) {
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
            segmentsByCamera.getOrPut(key(segment.cameraId, segment.cameraType)) { ArrayList() }.add(segment)
        }
    }

    private fun loadNegativeSegments(db: SQLiteDatabase) {
        val rows = ArrayList<Triple<Int, NegativeSegment, MutableList<Point>>>()
        db.rawQuery(
            "SELECT exclusion_id,camera_id,camera_type,source_kind,confidence,label " +
                "FROM exclusion_segments ORDER BY exclusion_id",
            null
        ).use { c ->
            while (c.moveToNext()) {
                val id = c.getInt(0)
                val points = ArrayList<Point>()
                val segment = NegativeSegment(
                    id = id,
                    cameraId = c.getString(1),
                    cameraType = c.getString(2),
                    source = c.getString(3),
                    confidence = c.getString(4),
                    label = c.getString(5),
                    points = points
                )
                rows += Triple(id, segment, points)
            }
        }

        val byId = rows.associateBy { it.first }
        db.rawQuery(
            "SELECT exclusion_id,seq,latitude,longitude FROM exclusion_points ORDER BY exclusion_id,seq",
            null
        ).use { c ->
            while (c.moveToNext()) {
                val id = c.getInt(0)
                byId[id]?.third?.add(Point(c.getDouble(2), c.getDouble(3)))
            }
        }

        for ((_, segment, _) in rows) {
            if (segment.points.size < 2) continue
            negativeSegmentsByCamera
                .getOrPut(key(segment.cameraId, segment.cameraType)) { ArrayList() }
                .add(segment)
        }
    }

    fun evaluate(
        cameraIds: List<String>,
        cameraType: String,
        lat: Double,
        lon: Double,
        vehicleBearingDeg: Float?
    ): Result? {
        if (!lat.isFinite() || !lon.isFinite()) return null

        val positives = ArrayList<Candidate>()
        val negatives = ArrayList<NegativeCandidate>()

        for (cameraId in cameraIds) {
            val positiveSegments = segmentsByCamera[key(cameraId, cameraType)].orEmpty()
            for (segment in positiveSegments) {
                val nearest = nearestToPolyline(lat, lon, segment.points) ?: continue
                val headingDelta = if (vehicleBearingDeg != null) {
                    angleDifference(vehicleBearingDeg, nearest.second)
                } else {
                    180f
                }
                positives += Candidate(
                    segment = segment,
                    corridorDistanceM = nearest.first,
                    headingDeltaDeg = headingDelta,
                    verdict = classifyPositive(nearest.first, headingDelta, vehicleBearingDeg != null)
                )
            }

            val explicitNegativeSegments = negativeSegmentsByCamera[key(cameraId, cameraType)].orEmpty()
            for (segment in explicitNegativeSegments) {
                val nearest = nearestToPolyline(lat, lon, segment.points) ?: continue
                val headingDelta = if (vehicleBearingDeg != null) {
                    angleDifference(vehicleBearingDeg, nearest.second)
                } else {
                    180f
                }
                negatives += NegativeCandidate(segment, nearest.first, headingDelta)
            }
        }

        if (positives.isEmpty() && negatives.isEmpty()) return null

        val positiveInCoverage = positives.count { it.corridorDistanceM <= COVERAGE_DISTANCE_M }
        val positiveStrong = choosePositive(positives, "support")
            ?: choosePositive(positives, "weak_support")

        // A field-proven positive approach always wins. This is the most important safety rule:
        // a negative route learned elsewhere must never veto a real approach that we have already
        // driven and validated.
        if (positiveStrong != null) {
            return positiveResult(positiveStrong, positives.size, positiveInCoverage, negatives.size)
        }

        // Explicit user-confirmed false-route evidence is only trusted when both position AND
        // travel direction match closely. This is intentionally much narrower than ordinary
        // positive coverage. Only this exact verdict is eligible for the guarded first-alert
        // veto in LocationService; every generic positive/conflict result stays shadow-only.
        val negativeMatch = if (vehicleBearingDeg != null) {
            negatives
                .asSequence()
                .filter {
                    it.corridorDistanceM <= NEGATIVE_ROUTE_DISTANCE_M &&
                        it.headingDeltaDeg <= NEGATIVE_ROUTE_HEADING_DEG
                }
                .minByOrNull { it.corridorDistanceM + it.headingDeltaDeg * 0.35f }
        } else {
            null
        }

        if (negativeMatch != null) {
            return Result(
                cameraId = negativeMatch.segment.cameraId,
                source = negativeMatch.segment.source,
                confidence = negativeMatch.segment.confidence,
                segmentId = -negativeMatch.segment.id,
                segmentsConsidered = positives.size,
                inCoverageSegments = positiveInCoverage,
                negativeSegmentsConsidered = negatives.size,
                corridorDistanceM = negativeMatch.corridorDistanceM,
                headingDeltaDeg = negativeMatch.headingDeltaDeg,
                verdict = "negative_route",
                evidenceKind = "explicit_negative",
                negativeLabel = negativeMatch.segment.label
            )
        }

        // No explicit negative match: preserve the v0.14 conservative positive policy exactly.
        val fallback = choosePositive(positives, "uncertain")
            ?: choosePositive(positives, "position_only")
            ?: choosePositive(positives, "conflict")
            ?: positives.minByOrNull { it.corridorDistanceM }

        if (fallback != null) {
            return positiveResult(fallback, positives.size, positiveInCoverage, negatives.size)
        }

        // Camera has only negative-route knowledge, but the vehicle is not on that learned false
        // route now. Treat as unknown/outside coverage, never as a conflict.
        val nearestNegative = negatives.minByOrNull { it.corridorDistanceM } ?: return null
        return Result(
            cameraId = nearestNegative.segment.cameraId,
            source = nearestNegative.segment.source,
            confidence = nearestNegative.segment.confidence,
            segmentId = -nearestNegative.segment.id,
            segmentsConsidered = 0,
            inCoverageSegments = 0,
            negativeSegmentsConsidered = negatives.size,
            corridorDistanceM = nearestNegative.corridorDistanceM,
            headingDeltaDeg = nearestNegative.headingDeltaDeg,
            verdict = "outside_coverage",
            evidenceKind = "explicit_negative",
            negativeLabel = nearestNegative.segment.label
        )
    }

    private fun positiveResult(
        candidate: Candidate,
        total: Int,
        inCoverage: Int,
        negativeCount: Int
    ): Result = Result(
        cameraId = candidate.segment.cameraId,
        source = candidate.segment.source,
        confidence = candidate.segment.confidence,
        segmentId = candidate.segment.id,
        segmentsConsidered = total,
        inCoverageSegments = inCoverage,
        negativeSegmentsConsidered = negativeCount,
        corridorDistanceM = candidate.corridorDistanceM,
        headingDeltaDeg = candidate.headingDeltaDeg,
        verdict = candidate.verdict,
        evidenceKind = "positive"
    )

    private fun choosePositive(candidates: List<Candidate>, verdict: String): Candidate? {
        return candidates
            .asSequence()
            .filter { it.verdict == verdict }
            .minByOrNull {
                it.corridorDistanceM + min(it.headingDeltaDeg, 180f) * 0.35f
            }
    }

    private fun classifyPositive(distanceM: Float, headingDeltaDeg: Float, bearingAvailable: Boolean): String {
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
