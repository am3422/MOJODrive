package com.mojotech.mojodrive

import android.Manifest
import android.app.*
import android.content.ContentValues
import android.content.Context
import android.content.Intent
import android.content.pm.PackageManager
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.location.GnssStatus
import android.location.Location
import android.location.LocationListener
import android.location.LocationManager
import android.media.AudioManager
import android.media.ToneGenerator
import android.net.Uri
import android.os.*
import android.provider.MediaStore
import android.widget.Toast
import com.google.android.gms.common.ConnectionResult
import com.google.android.gms.common.GoogleApiAvailability
import com.google.android.gms.location.FusedLocationProviderClient
import com.google.android.gms.location.LocationCallback
import com.google.android.gms.location.LocationRequest
import com.google.android.gms.location.LocationResult
import com.google.android.gms.location.LocationServices
import com.google.android.gms.location.Priority
import java.io.BufferedWriter
import java.io.File
import java.io.OutputStreamWriter
import java.text.SimpleDateFormat
import java.util.ArrayDeque
import java.util.Date
import java.util.Locale
import kotlin.math.*

class LocationService : Service(), LocationListener, SensorEventListener {

    companion object {
        const val EXTRA_THRESHOLD_KMH = "threshold_kmh"
        const val ACTION_MARK_EVENT = "com.mojotech.mojodrive.MARK_EVENT"
        const val ACTION_STOP_TRIP = "com.mojotech.mojodrive.STOP_TRIP"
        const val ACTION_TEST_ALERT = "com.mojotech.mojodrive.TEST_ALERT"

        private const val CHANNEL_ID = "mojo_drive_tracking"
        private const val NOTIFICATION_ID = 100

        private const val LOCATION_STALE_MS = 8000L
        private const val LOCATION_RECOVERY_RESET_MS = 3000L
        private const val GPS_CAMERA_BRIDGE_START_MS = 1500L
        private const val GPS_CAMERA_BRIDGE_MAX_MS = 12_000L
        private const val FUSION_BRIDGE_MS = 12_000L
        private const val SPEED_DISPLAY_STALE_MS = 4_000L
        private const val SPEED_HARD_INVALIDATE_MS = 12_000L
        private const val STATIONARY_RAW_SPEED_KMH = 2.5f
        private const val STATIONARY_DERIVED_SPEED_KMH = 3.5f
        private const val STATIONARY_CONFIRM_HITS = 2
        private const val BEARING_HOLD_MS = 8000L

        private const val LOCATION_FALLBACK_TRIGGER_MS = 6000L
        private const val LOCATION_STALL_LOG_MS = 30_000L

        private const val ROAD_IMPACT_COOLDOWN_MS = 1200L
        private const val ROAD_IMPACT_SHOCK_MPS2 = 3.2f
        private const val OVERSPEED_REPEAT_MS = 15_000L

        private const val CAMERA_SEARCH_M = 2300f
        private const val CAMERA_CONFIRM_EXTRA_M = 500f
        private const val CAMERA_CLUSTER_BASE_M = 40f
        private const val CAMERA_CLUSTER_EXTENDED_M = 110f
        private const val CAMERA_CLUSTER_AXIS_DELTA_DEG = 18f
        private const val CAMERA_CLUSTER_DIRECTED_DELTA_DEG = 22f
        private const val CAMERA_DIAG_NEAR_M = 950f
        private const val CAMERA_DIAG_INTERVAL_MS = 5000L

        private const val PASS_BEHIND_DEG = 100f
        private const val PASS_DISTANCE_GROWTH_M = 25f
        private const val NEAR_PASS_AUDIT_M = 140f

        private const val GLOBAL_ALERT_MIN_GAP_MS = 350L
        private const val FIRST_ALERT_FORCE_MS = 1200L

        private const val SENSOR_PROCESS_PERIOD_NS = 20_000_000L
        private const val SENSOR_HEALTH_LOG_MS = 10_000L
        private const val LIVE_LOG_FLUSH_MS = 1000L
    }

    private enum class TrackPhase {
        TRACKING, CONFIRMED, PASSED, ABANDONED
    }

    private data class CameraCluster(
        val key: String,
        val camera: CameraPoint,
        val memberIds: List<String>
    )

    private data class CameraGeometry(
        val cluster: CameraCluster,
        val directDistanceM: Float,
        val vehicleForwardM: Float,
        val vehicleCrossM: Float,
        val bearingToCameraDeg: Float,
        val forwardDeltaDeg: Float,
        val roadCrossM: Float,
        val curveCrossM: Float,
        val roadAxisDeltaDeg: Float,
        val roadDirectionDeltaDeg: Float,
        val hardCrossLimitM: Float,
        val hardCrossOk: Boolean,
        val directionGateOk: Boolean,
        val hardGeometryOk: Boolean,
        val warningDistanceM: Float,
        val ttcS: Float,
        val candidateLimit: Int?,
        val approachDeltaM: Float,
        val approachRateMps: Float,
        val matchScore: Int,
        val requiredScore: Int,
        val requiredHits: Int,
        val matchReason: String
    )

    private data class CameraTrackState(
        var phase: TrackPhase = TrackPhase.TRACKING,
        var confirmHits: Int = 0,
        var approachHits: Int = 0,
        var awayHits: Int = 0,
        var hardBadHits: Int = 0,
        var warned: Boolean = false,
        var urgentLogged: Boolean = false,
        var nearPassAudited: Boolean = false,
        var alertLevel: Int = 0,
        var lastDiagElapsed: Long = 0L,
        var lastSeenElapsed: Long = 0L,
        var transitionElapsed: Long = 0L,
        var lastDistanceM: Float = Float.MAX_VALUE,
        var minDistanceM: Float = Float.MAX_VALUE,
        var lastGeometry: CameraGeometry? = null,
        var lastGeometryElapsed: Long = 0L,
        var bridgeLogged: Boolean = false,
        var warningStartedElapsed: Long = 0L,
        var firstAlertDelivered: Boolean = false,
        var lastPulseElapsed: Long = 0L,
        var missedDuringGapLogged: Boolean = false
    )

    private data class AlertRequest(
        val geometry: CameraGeometry,
        val state: CameraTrackState,
        val level: Int,
        val bridge: Boolean
    )

    private lateinit var locationManager: LocationManager
    private lateinit var sensorManager: SensorManager
    private lateinit var prefs: android.content.SharedPreferences
    private lateinit var powerManager: PowerManager
    private lateinit var audioManager: AudioManager

    private var accelerometer: Sensor? = null
    private var gyroscope: Sensor? = null
    private var linearAcceleration: Sensor? = null
    private var rotationVector: Sensor? = null
    private var wakeLock: PowerManager.WakeLock? = null

    private var toneLow: ToneGenerator? = null
    private var toneMid: ToneGenerator? = null
    private var toneHigh: ToneGenerator? = null

    private var cameras: List<CameraPoint> = emptyList()
    private var cameraClusters: List<CameraCluster> = emptyList()
    private val cameraStates = HashMap<String, CameraTrackState>()

    private val handler = Handler(Looper.getMainLooper())
    private val speedSamples = ArrayDeque<Float>()
    private val derivedSpeedSamples = ArrayDeque<Float>()
    private val rotationMatrix = FloatArray(9)

    private var rotationReady = false
    private var fusionInitialized = false
    private var fusionSpeedMps = 0f
    private var fusionVariance = 4f
    private var lastFusionSensorNs = 0L
    private var latestGpsFilteredKmh = Float.NaN
    private var latestDerivedSpeedKmh = Float.NaN
    private var latestForwardAccelMps2 = 0f
    private var fusionActive = false
    private var imuBridgeActive = false
    private var latestSpeedEstimateReliable = false
    private var speedDisplayValid = false
    private var stationarySpeedHits = 0
    private var speedInvalidationLogged = false

    private var previousPositionForSpeed: Location? = null
    private var previousPositionElapsedMs = 0L

    private var manualThresholdKmh = 80
    private var activeLimitKmh = 80
    private var lastAlertLimitKmh = 80
    private var overspeedActive = false
    private var lastOverspeedAlertElapsed = 0L

    private var globalLastCameraPulseElapsed = 0L
    private var globalLastCameraKey = ""
    private var cameraAlertZoneActive = false

    private var lastRoadImpactElapsed = 0L
    private var latestGyroMagnitude = 0f
    private var lastAccelProcessNs = 0L
    private var lastGyroProcessNs = 0L
    private var lastLinearProcessNs = 0L
    private var lastRotationProcessNs = 0L
    private var accelProcessedCount = 0
    private var gyroProcessedCount = 0
    private var linearProcessedCount = 0
    private var rotationProcessedCount = 0
    private var lastSensorHealthElapsed = 0L

    private lateinit var logFile: File
    private lateinit var logName: String
    private var loggingStarted = false
    private var userStopRequested = false
    private var liveLogUri: Uri? = null
    private var liveWriter: BufferedWriter? = null
    private var lastLiveFlushElapsed = 0L

    private lateinit var fusedLocationClient: FusedLocationProviderClient
    private var fusedUpdatesRegistered = false
    private var legacyLocationUpdatesRegistered = false
    private var networkFallbackRegistered = false
    private var sensorsRegistered = false
    private var gnssRegistered = false
    private var serviceTrackingActive = false
    private var lastLocationRequestElapsedMs = 0L
    private var lastLocationStallLogElapsedMs = 0L
    private var locationStallCount = 0
    private var lastProcessedLocationRealtimeNanos = 0L

    @Volatile private var latestLat = Double.NaN
    @Volatile private var latestLon = Double.NaN
    @Volatile private var latestRawSpeedKmh = Float.NaN
    @Volatile private var latestSpeedKmh = 0f
    @Volatile private var latestBearing = -1f
    @Volatile private var latestBearingValid = false
    @Volatile private var latestAccuracy = -1f
    @Volatile private var latestProvider = "--"

    @Volatile private var lastGpsCallbackElapsedMs = 0L
    @Volatile private var lastValidLocationElapsedMs = 0L
    @Volatile private var lastValidSpeedElapsedMs = 0L
    @Volatile private var lastSpeedEstimateElapsedMs = 0L
    @Volatile private var lastBearingElapsedMs = 0L
    @Volatile private var gpsStale = true
    @Volatile private var recoveredLocationGapMs = 0L

    @Volatile private var gnssSatellites = 0
    @Volatile private var gnssUsedInFix = 0
    @Volatile private var lastGnssStatusElapsedMs = 0L

    @Volatile private var activeCameraId = ""
    @Volatile private var activeCameraType = ""
    @Volatile private var activeCameraDistanceM = -1f
    @Volatile private var activeCameraSpeedLimit: Int? = null
    @Volatile private var activeCameraRoadName = ""
    @Volatile private var activeCameraCurveCrossM = -1f
    @Volatile private var activeCameraForwardDelta = -1f
    @Volatile private var activeCameraAlertLevel = 0
    @Volatile private var activeCameraTtcS = -1f
    @Volatile private var activeCameraWarningDistanceM = -1f
    @Volatile private var confirmedCameraCount = 0

    private val gnssCallback = object : GnssStatus.Callback() {
        override fun onStarted() {
            if (loggingStarted) appendMarker("GNSS_STARTED")
        }

        override fun onStopped() {
            gnssSatellites = 0
            gnssUsedInFix = 0
            if (loggingStarted) appendMarker("GNSS_STOPPED")
        }

        override fun onFirstFix(ttffMillis: Int) {
            if (loggingStarted) appendMarker("GNSS_FIRST_FIX", "ttff_ms=$ttffMillis")
        }

        override fun onSatelliteStatusChanged(status: GnssStatus) {
            var used = 0
            for (i in 0 until status.satelliteCount) {
                if (status.usedInFix(i)) used++
            }
            gnssSatellites = status.satelliteCount
            gnssUsedInFix = used
            lastGnssStatusElapsedMs = SystemClock.elapsedRealtime()
        }
    }

    private val healthTicker = object : Runnable {
        override fun run() {
            updateLocationHealth()
            monitorLocationPipeline()
            maybeLogSensorHealth()
            handler.postDelayed(this, 500L)
        }
    }

    override fun onCreate() {
        super.onCreate()

        prefs = getSharedPreferences("mojo_drive", MODE_PRIVATE)
        locationManager = getSystemService(Context.LOCATION_SERVICE) as LocationManager
        sensorManager = getSystemService(Context.SENSOR_SERVICE) as SensorManager
        powerManager = getSystemService(Context.POWER_SERVICE) as PowerManager
        audioManager = getSystemService(Context.AUDIO_SERVICE) as AudioManager
        fusedLocationClient = LocationServices.getFusedLocationProviderClient(this)

        accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
        gyroscope = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
        linearAcceleration = sensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION)
        rotationVector = sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR)

        toneLow = ToneGenerator(AudioManager.STREAM_ALARM, 55)
        toneMid = ToneGenerator(AudioManager.STREAM_ALARM, 80)
        toneHigh = ToneGenerator(AudioManager.STREAM_ALARM, 100)

        try {
            cameras = CameraDatabase(this).loadAll()
            cameraClusters = buildCameraClusters(cameras)
        } catch (_: Exception) {
            cameras = emptyList()
            cameraClusters = emptyList()
        }

        prefs.edit()
            .putBoolean("accel_available", accelerometer != null)
            .putBoolean("gyro_available", gyroscope != null)
            .putBoolean("linear_accel_available", linearAcceleration != null)
            .putBoolean("rotation_vector_available", rotationVector != null)
            .putInt("camera_count", cameras.size)
            .putInt("camera_cluster_count", cameraClusters.size)
            .apply()

        createNotificationChannel()
        acquireWakeLock()
        handler.post(healthTicker)
    }

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        when (intent?.action) {
            ACTION_TEST_ALERT -> {
                runStandaloneAlertTest()
                stopSelf(startId)
                return START_NOT_STICKY
            }

            ACTION_MARK_EVENT -> {
                if (loggingStarted) {
                    appendMarker("USER_ROAD_EVENT")
                    vibratePattern(longArrayOf(0, 100), intArrayOf(0, 180))
                }
                return START_STICKY
            }

            ACTION_STOP_TRIP -> {
                userStopRequested = true
                serviceTrackingActive = false
                if (loggingStarted || prefs.getBoolean("trip_active", false)) {
                    ensureTripLoadedForStop()
                    appendMarker("STOP")
                    finalizePersistentTrip()
                }
                stopForeground(STOP_FOREGROUND_REMOVE)
                stopSelf()
                return START_NOT_STICKY
            }
        }

        manualThresholdKmh = intent?.getIntExtra(
            EXTRA_THRESHOLD_KMH,
            prefs.getInt("threshold_kmh", 80)
        ) ?: prefs.getInt("threshold_kmh", 80)

        activeLimitKmh = manualThresholdKmh
        lastAlertLimitKmh = manualThresholdKmh
        userStopRequested = false
        serviceTrackingActive = true

        if (!loggingStarted && !resumeInterruptedTrip()) {
            startNewTripLog()
        }

        startForeground(NOTIFICATION_ID, makeNotification("Waiting for valid GPS location…"))
        prefs.edit().putBoolean("running", true).apply()

        startLocationPipeline()
        registerGnssStatus()
        startSensorUpdates()

        if (loggingStarted) {
            appendMarker(
                "CAMERA_DB_READY",
                "raw=${cameras.size};clusters=${cameraClusters.size};" +
                    "db=shiraz_cameras_v11;engine=hybrid_location_curve_aware_v12"
            )
        }

        return START_STICKY
    }

    private fun buildCameraClusters(input: List<CameraPoint>): List<CameraCluster> {
        val remaining = input.toMutableList()
        val output = ArrayList<CameraCluster>()

        while (remaining.isNotEmpty()) {
            val seed = remaining.removeAt(0)
            val members = mutableListOf(seed)

            var changed = true
            while (changed) {
                changed = false
                val iterator = remaining.iterator()
                while (iterator.hasNext()) {
                    val candidate = iterator.next()
                    if (members.any { camerasClusterCompatible(it, candidate) }) {
                        members += candidate
                        iterator.remove()
                        changed = true
                    }
                }
            }

            val representative = members.maxByOrNull { cameraMetadataScore(it) } ?: seed
            val averageLat = members.map { it.latitude }.average()
            val averageLon = members.map { it.longitude }.average()
            val synthetic = representative.copy(latitude = averageLat, longitude = averageLon)
            val ids = members.map { it.id }.distinct().sorted()

            output += CameraCluster(
                key = representative.type + ":" + ids.joinToString("+"),
                camera = synthetic,
                memberIds = ids
            )
        }

        return output
    }

    private fun camerasClusterCompatible(a: CameraPoint, b: CameraPoint): Boolean {
        if (a.type != b.type) return false

        val results = FloatArray(1)
        Location.distanceBetween(a.latitude, a.longitude, b.latitude, b.longitude, results)
        val distance = results[0]
        if (distance > CAMERA_CLUSTER_EXTENDED_M) return false

        val aRb = a.roadBearing
        val bRb = b.roadBearing

        if (distance <= CAMERA_CLUSTER_BASE_M) {
            if (aRb == null || bRb == null) return true
            return bearingsClusterCompatible(a, b)
        }

        // Extended clustering is deliberately strict: it exists only to collapse
        // duplicate database records for one physical camera, not nearby distinct cameras.
        if (aRb == null || bRb == null) return false
        if (!bearingsClusterCompatible(a, b)) return false

        val aLimit = a.speedLimit
        val bLimit = b.speedLimit
        if (aLimit != null && bLimit != null && abs(aLimit - bLimit) > 5) return false

        val aRoad = normalizeRoadName(a.roadName)
        val bRoad = normalizeRoadName(b.roadName)
        if (aRoad.isNotBlank() && bRoad.isNotBlank() && aRoad != bRoad) return false

        return true
    }

    private fun bearingsClusterCompatible(a: CameraPoint, b: CameraPoint): Boolean {
        val aRb = a.roadBearing ?: return true
        val bRb = b.roadBearing ?: return true
        val aDirected = hasReliableDirectedBearing(a)
        val bDirected = hasReliableDirectedBearing(b)

        return if (aDirected && bDirected) {
            angleDifference(aRb, bRb) <= CAMERA_CLUSTER_DIRECTED_DELTA_DEG
        } else {
            val axisDelta = min(
                angleDifference(aRb, bRb),
                angleDifference(aRb, normalizeBearing(bRb + 180f))
            )
            axisDelta <= CAMERA_CLUSTER_AXIS_DELTA_DEG
        }
    }

    private fun normalizeRoadName(value: String): String =
        value.trim().lowercase(Locale.US).replace(Regex("\\s+"), " ")

    private fun hasReliableDirectedBearing(camera: CameraPoint): Boolean {
        if (!camera.oneway || camera.roadBearing == null) return false
        val confidence = camera.directionConfidence.lowercase(Locale.US)
        return confidence == "high" || confidence == "medium"
    }

    private fun cameraMetadataScore(camera: CameraPoint): Int {
        var score = 0
        if (validDynamicLimit(camera) != null) score += 5
        if (camera.roadBearing != null) score += 3
        when (camera.directionConfidence.lowercase(Locale.US)) {
            "high" -> score += 3
            "medium" -> score += 2
        }
        if ((camera.roadDistanceM ?: 999.0) <= 30.0) score += 2
        if (camera.roadName.isNotBlank()) score += 1
        return score
    }

    private fun acquireWakeLock() {
        try {
            wakeLock = powerManager.newWakeLock(
                PowerManager.PARTIAL_WAKE_LOCK,
                "MOJODrive::Tracking"
            ).apply {
                setReferenceCounted(false)
                acquire()
            }
        } catch (_: Exception) {}
    }

    private fun runStandaloneAlertTest() {
        val vibrator = getSystemService(Context.VIBRATOR_SERVICE) as Vibrator
        val vibratorPresent = try { vibrator.hasVibrator() } catch (_: Exception) { false }
        val toneOk = try {
            toneHigh?.startTone(ToneGenerator.TONE_PROP_BEEP2, 550) ?: false
        } catch (_: Exception) { false }

        var vibrationDispatched = false
        if (vibratorPresent) {
            try {
                vibratePattern(
                    longArrayOf(0, 220, 90, 220),
                    intArrayOf(0, 255, 0, 255)
                )
                vibrationDispatched = true
            } catch (_: Exception) {}
        }

        val alarmVolume = try {
            audioManager.getStreamVolume(AudioManager.STREAM_ALARM)
        } catch (_: Exception) { -1 }

        val alarmMax = try {
            audioManager.getStreamMaxVolume(AudioManager.STREAM_ALARM)
        } catch (_: Exception) { -1 }

        prefs.edit()
            .putBoolean("last_test_tone_ok", toneOk)
            .putBoolean("last_test_vibrator_present", vibratorPresent)
            .putBoolean("last_test_vibration_dispatched", vibrationDispatched)
            .putInt("last_test_alarm_volume", alarmVolume)
            .putInt("last_test_alarm_max", alarmMax)
            .apply()

        Toast.makeText(
            this,
            "Alert test: tone ${if (toneOk) "OK" else "FAILED"}" +
                " • vibration ${if (vibrationDispatched) "DISPATCHED" else "FAILED"}" +
                " • alarm volume $alarmVolume/$alarmMax",
            Toast.LENGTH_LONG
        ).show()
    }

    private fun csvHeader(): String =
        "wall_time_ms,iso_time,record_type,lat,lon,provider,gps_age_ms,gps_speed_age_ms," +
            "gps_callback_age_ms,gps_stale,location_valid,speed_valid,raw_speed_kmh,gps_filtered_kmh," +
            "derived_speed_kmh,fused_speed_kmh,forward_accel_mps2,fusion_active,imu_bridge," +
            "bearing_deg,bearing_valid,gps_accuracy_m,gnss_satellites,gnss_used_in_fix," +
            "camera_id,camera_type,camera_lat,camera_lon,camera_direct_distance_m,camera_forward_m," +
            "camera_vehicle_cross_m,camera_road_cross_m,camera_curve_cross_m,camera_bearing_to_deg," +
            "camera_forward_delta_deg,camera_axis_delta_deg,camera_direction_delta_deg," +
            "camera_hard_geometry_ok,camera_match_score,camera_required_score,camera_confirmed," +
            "camera_alert_level,camera_ttc_s,camera_warning_distance_m,camera_limit_kmh,active_limit_kmh," +
            "x,y,z,magnitude,sensor_timestamp_ns,details\n"

    private fun startNewTripLog() {
        val stamp = SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(Date())
        logName = "MOJODrive_Trip_$stamp.csv"
        logFile = File(filesDir, logName)
        logFile.writeText(csvHeader())

        liveLogUri = createPublicLiveLog(logName)
        openLiveWriter()
        try {
            liveWriter?.write(csvHeader())
            liveWriter?.flush()
        } catch (_: Exception) {}

        prefs.edit()
            .putBoolean("trip_active", true)
            .putString("active_log_name", logName)
            .putString("active_log_uri", liveLogUri?.toString())
            .apply()

        loggingStarted = true
        appendMarker(
            "START",
            "persistent_live_log=true;camera_engine=hybrid_location_curve_aware_v12;raw_sensor_csv=false"
        )
    }

    private fun resumeInterruptedTrip(): Boolean {
        if (!prefs.getBoolean("trip_active", false)) return false
        val name = prefs.getString("active_log_name", null) ?: return false
        val internal = File(filesDir, name)
        if (!internal.exists() || internal.length() == 0L) return false

        val header = try { internal.bufferedReader().use { it.readLine() ?: "" } } catch (_: Exception) { "" }
        if (!header.contains("gps_speed_age_ms") || !header.contains("camera_curve_cross_m")) {
            prefs.edit()
                .putBoolean("trip_active", false)
                .putString("last_log_name", name)
                .remove("active_log_uri")
                .remove("active_log_name")
                .apply()
            return false
        }

        logName = name
        logFile = internal
        liveLogUri = prefs.getString("active_log_uri", null)?.let {
            try { Uri.parse(it) } catch (_: Exception) { null }
        }

        if (liveLogUri != null) {
            try {
                contentResolver.openOutputStream(liveLogUri!!, "w")?.use { out ->
                    logFile.inputStream().use { input -> input.copyTo(out) }
                }
            } catch (_: Exception) {
                liveLogUri = null
            }
        }

        if (liveLogUri == null) {
            liveLogUri = createPublicLiveLog(logName)
            if (liveLogUri != null) {
                try {
                    contentResolver.openOutputStream(liveLogUri!!, "w")?.use { out ->
                        logFile.inputStream().use { input -> input.copyTo(out) }
                    }
                } catch (_: Exception) {}
                prefs.edit().putString("active_log_uri", liveLogUri.toString()).apply()
            }
        }

        openLiveWriter()
        loggingStarted = true
        appendMarker("RECOVERED_AFTER_RESTART", "previous_session_was_interrupted=true")
        return true
    }

    private fun ensureTripLoadedForStop() {
        if (!loggingStarted) resumeInterruptedTrip()
    }

    private fun createPublicLiveLog(name: String): Uri? {
        if (Build.VERSION.SDK_INT < 29) return null
        return try {
            val values = ContentValues().apply {
                put(MediaStore.Downloads.DISPLAY_NAME, name)
                put(MediaStore.Downloads.MIME_TYPE, "text/csv")
                put(MediaStore.Downloads.RELATIVE_PATH, Environment.DIRECTORY_DOWNLOADS + "/MOJODrive")
            }
            contentResolver.insert(MediaStore.Downloads.EXTERNAL_CONTENT_URI, values)
        } catch (_: Exception) {
            null
        }
    }

    private fun openLiveWriter() {
        closeLiveWriter()
        val uri = liveLogUri ?: return
        try {
            liveWriter = BufferedWriter(OutputStreamWriter(contentResolver.openOutputStream(uri, "wa")!!))
            lastLiveFlushElapsed = SystemClock.elapsedRealtime()
        } catch (_: Exception) {
            liveWriter = null
        }
    }

    private fun closeLiveWriter() {
        try { liveWriter?.flush() } catch (_: Exception) {}
        try { liveWriter?.close() } catch (_: Exception) {}
        liveWriter = null
    }

    private fun finalizePersistentTrip() {
        if (!loggingStarted) {
            prefs.edit().putBoolean("trip_active", false).putBoolean("running", false).apply()
            return
        }

        closeLiveWriter()
        prefs.edit()
            .putBoolean("trip_active", false)
            .putBoolean("running", false)
            .putString("last_log_name", logName)
            .remove("active_log_uri")
            .remove("active_log_name")
            .apply()

        loggingStarted = false
        Toast.makeText(
            this,
            "Trip log finalized: Downloads/MOJODrive/$logName",
            Toast.LENGTH_LONG
        ).show()
    }

    private val fusedLocationCallback = object : LocationCallback() {
        override fun onLocationResult(result: LocationResult) {
            for (location in result.locations) {
                processLocation(location, "fused:${location.provider ?: "unknown"}")
            }
        }
    }

    private fun startLocationPipeline() {
        if (
            checkSelfPermission(Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED &&
            checkSelfPermission(Manifest.permission.ACCESS_COARSE_LOCATION) != PackageManager.PERMISSION_GRANTED
        ) {
            return
        }

        lastLocationRequestElapsedMs = SystemClock.elapsedRealtime()

        val playServicesAvailable =
            GoogleApiAvailability.getInstance().isGooglePlayServicesAvailable(this) == ConnectionResult.SUCCESS

        if (playServicesAvailable) {
            requestFusedLocationUpdates()
        } else {
            appendMarker("LOCATION_PRIMARY_UNAVAILABLE", "source=fused;reason=play_services_unavailable")
            startLegacyLocationFallback("no_play_services")
        }
    }

    private fun requestFusedLocationUpdates() {
        if (fusedUpdatesRegistered) return
        if (
            checkSelfPermission(Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED &&
            checkSelfPermission(Manifest.permission.ACCESS_COARSE_LOCATION) != PackageManager.PERMISSION_GRANTED
        ) return

        try {
            val request = LocationRequest.Builder(Priority.PRIORITY_HIGH_ACCURACY, 1000L)
                .setMinUpdateIntervalMillis(500L)
                .setMaxUpdateDelayMillis(1000L)
                .setMinUpdateDistanceMeters(0f)
                .setWaitForAccurateLocation(false)
                .build()

            fusedLocationClient.requestLocationUpdates(
                request,
                fusedLocationCallback,
                Looper.getMainLooper()
            ).addOnSuccessListener {
                fusedUpdatesRegistered = true
                if (loggingStarted) appendMarker("LOCATION_REQUEST", "source=fused;reason=service_start")
            }.addOnFailureListener { e ->
                fusedUpdatesRegistered = false
                if (loggingStarted) {
                    appendMarker(
                        "LOCATION_REQUEST_FAILED",
                        "source=fused;error=${e.javaClass.simpleName}"
                    )
                }
                startLegacyLocationFallback("fused_request_failed")
            }
        } catch (e: Exception) {
            fusedUpdatesRegistered = false
            if (loggingStarted) {
                appendMarker(
                    "LOCATION_REQUEST_FAILED",
                    "source=fused;error=${e.javaClass.simpleName}"
                )
            }
            startLegacyLocationFallback("fused_exception")
        }
    }

    private fun startLegacyLocationFallback(reason: String) {
        if (legacyLocationUpdatesRegistered) return
        if (
            checkSelfPermission(Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED &&
            checkSelfPermission(Manifest.permission.ACCESS_COARSE_LOCATION) != PackageManager.PERMISSION_GRANTED
        ) return

        var registeredAny = false

        try {
            if (locationManager.isProviderEnabled(LocationManager.GPS_PROVIDER)) {
                locationManager.requestLocationUpdates(
                    LocationManager.GPS_PROVIDER,
                    700L,
                    0f,
                    this
                )
                registeredAny = true
            }
        } catch (e: Exception) {
            if (loggingStarted) appendMarker(
                "LOCATION_REQUEST_FAILED",
                "source=legacy_gps;error=${e.javaClass.simpleName}"
            )
        }

        try {
            if (locationManager.isProviderEnabled(LocationManager.NETWORK_PROVIDER)) {
                locationManager.requestLocationUpdates(
                    LocationManager.NETWORK_PROVIDER,
                    1000L,
                    0f,
                    this
                )
                networkFallbackRegistered = true
                registeredAny = true
            }
        } catch (e: Exception) {
            if (loggingStarted) appendMarker(
                "LOCATION_REQUEST_FAILED",
                "source=legacy_network;error=${e.javaClass.simpleName}"
            )
        }

        legacyLocationUpdatesRegistered = registeredAny
        if (registeredAny && loggingStarted) {
            appendMarker(
                "LOCATION_FALLBACK_ENABLED",
                "reason=$reason;gps=${safeProviderEnabled(LocationManager.GPS_PROVIDER)};" +
                    "network=${safeProviderEnabled(LocationManager.NETWORK_PROVIDER)}"
            )
        }
    }

    private fun safeProviderEnabled(provider: String): Boolean =
        try { locationManager.isProviderEnabled(provider) } catch (_: Exception) { false }

    private fun registerGnssStatus() {
        if (gnssRegistered) return
        if (checkSelfPermission(Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED) return
        try {
            gnssRegistered = locationManager.registerGnssStatusCallback(gnssCallback, handler)
        } catch (_: Exception) {
            gnssRegistered = false
        }
    }

    private fun startSensorUpdates() {
        if (sensorsRegistered) return
        accelerometer?.let { sensorManager.registerListener(this, it, 20_000) }
        gyroscope?.let { sensorManager.registerListener(this, it, 20_000) }
        linearAcceleration?.let { sensorManager.registerListener(this, it, 20_000) }
        rotationVector?.let { sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME) }
        sensorsRegistered = true
    }

    private fun monitorLocationPipeline() {
        if (!serviceTrackingActive) return
        val now = SystemClock.elapsedRealtime()
        val callbackAge = currentGpsCallbackAgeMs()
        val sinceRequest = now - lastLocationRequestElapsedMs

        val stalled =
            if (lastGpsCallbackElapsedMs <= 0L) {
                sinceRequest >= LOCATION_FALLBACK_TRIGGER_MS
            } else {
                callbackAge >= LOCATION_FALLBACK_TRIGGER_MS
            }

        if (stalled && !legacyLocationUpdatesRegistered) {
            startLegacyLocationFallback("primary_stall")
        }

        if (
            stalled &&
            now - lastLocationStallLogElapsedMs >= LOCATION_STALL_LOG_MS
        ) {
            lastLocationStallLogElapsedMs = now
            locationStallCount++
            appendMarker(
                "LOCATION_STALL",
                "callbackAgeMs=$callbackAge;sinceRequestMs=$sinceRequest;count=$locationStallCount;" +
                    "fused=$fusedUpdatesRegistered;legacy=$legacyLocationUpdatesRegistered;" +
                    "gpsEnabled=${safeProviderEnabled(LocationManager.GPS_PROVIDER)};" +
                    "networkEnabled=${safeProviderEnabled(LocationManager.NETWORK_PROVIDER)};" +
                    "satellites=$gnssSatellites;used=$gnssUsedInFix"
            )
        }
    }

    override fun onLocationChanged(location: Location) {
        processLocation(location, "legacy:${location.provider ?: "unknown"}")
    }

    private fun processLocation(location: Location, source: String) {
        val now = SystemClock.elapsedRealtime()

        // Fused and legacy providers can report the same underlying fix. Keep only newer fixes.
        val realtimeNanos = location.elapsedRealtimeNanos
        if (realtimeNanos > 0L && realtimeNanos <= lastProcessedLocationRealtimeNanos) {
            return
        }
        if (realtimeNanos > 0L) {
            lastProcessedLocationRealtimeNanos = realtimeNanos
        }

        val previousCallback = lastGpsCallbackElapsedMs
        lastGpsCallbackElapsedMs = now

        val ageMs = locationAgeMs(location)
        val accuracy = if (location.hasAccuracy()) location.accuracy else -1f
        val rawSpeed = if (location.hasSpeed()) location.speed * 3.6f else Float.NaN
        val rawLat = location.latitude
        val rawLon = location.longitude

        val isRawGps = location.provider == LocationManager.GPS_PROVIDER
        val maxLocationAccuracy = if (isRawGps) 50f else 80f

        val locationValid =
            ageMs in 0L..5000L &&
                (!location.hasAccuracy() || location.accuracy <= maxLocationAccuracy) &&
                rawLat.isFinite() &&
                rawLon.isFinite() &&
                rawLat in -90.0..90.0 &&
                rawLon in -180.0..180.0

        if (!locationValid) {
            appendRow(
                "LOCATION_REJECTED",
                Float.NaN, Float.NaN, Float.NaN, Float.NaN,
                0L,
                false,
                false,
                "source=$source;reason=${locationRejectionReason(location, ageMs)};" +
                    "rawLat=${String.format(Locale.US, "%.7f", rawLat)};" +
                    "rawLon=${String.format(Locale.US, "%.7f", rawLon)};" +
                    "rawAcc=${String.format(Locale.US, "%.1f", accuracy)};" +
                    "callbackGapMs=${if (previousCallback > 0L) now - previousCallback else -1L}",
                null
            )
            updatePrefs()
            updateNotification()
            return
        }

        val locationGap =
            if (lastValidLocationElapsedMs > 0L) now - lastValidLocationElapsedMs else 0L
        recoveredLocationGapMs = locationGap

        if (locationGap >= LOCATION_RECOVERY_RESET_MS) {
            prepareForLocationGapRecovery(locationGap)
        }

        val previousForBearing = previousPositionForSpeed?.let { Location(it) }
        val previousForBearingTime = previousPositionElapsedMs

        latestLat = rawLat
        latestLon = rawLon
        latestProvider = source
        latestAccuracy = accuracy
        latestRawSpeedKmh = rawSpeed
        lastValidLocationElapsedMs = now
        gpsStale = false

        latestDerivedSpeedKmh = updateDerivedSpeed(location, now)

        val derivedBearing =
            if (previousForBearing != null && previousForBearingTime > 0L) {
                val dt = (now - previousForBearingTime) / 1000f
                val prevAcc = if (previousForBearing.hasAccuracy()) previousForBearing.accuracy else 50f
                val curAcc = if (location.hasAccuracy()) location.accuracy else 50f
                val moved = previousForBearing.distanceTo(location)
                if (dt in 0.4f..5.0f && prevAcc <= 35f && curAcc <= 35f && moved >= 5f) {
                    normalizeBearing(previousForBearing.bearingTo(location))
                } else {
                    null
                }
            } else {
                null
            }

        val speedAccuracyOk =
            if (Build.VERSION.SDK_INT >= 26 && location.hasSpeedAccuracy()) {
                location.speedAccuracyMetersPerSecond <= 5.5f
            } else {
                true
            }

        val speedValid =
            location.hasSpeed() &&
                speedAccuracyOk &&
                rawSpeed in 0f..250f

        val bearingSupportSpeed = when {
            speedValid -> rawSpeed
            latestDerivedSpeedKmh.isFinite() -> latestDerivedSpeedKmh
            else -> latestSpeedKmh
        }

        if (location.hasBearing() && bearingSupportSpeed >= 5f) {
            latestBearing = normalizeBearing(location.bearing)
            latestBearingValid = true
            lastBearingElapsedMs = now
        } else if (derivedBearing != null && bearingSupportSpeed >= 5f) {
            latestBearing = derivedBearing
            latestBearingValid = true
            lastBearingElapsedMs = now
        } else {
            latestBearingValid =
                lastBearingElapsedMs > 0L &&
                    now - lastBearingElapsedMs <= BEARING_HOLD_MS
        }

        val priorGpsSpeedAge = currentGpsSpeedAgeMs()

        if (speedValid) {
            if (priorGpsSpeedAge < 0L || priorGpsSpeedAge > 5000L) {
                speedSamples.clear()
            }

            latestGpsFilteredKmh = filteredSpeed(rawSpeed)
            val gpsSpeedAccuracyMps =
                if (Build.VERSION.SDK_INT >= 26 && location.hasSpeedAccuracy()) {
                    location.speedAccuracyMetersPerSecond.coerceIn(0.3f, 5.5f)
                } else {
                    2.0f
                }

            if (!fusionInitialized || priorGpsSpeedAge < 0L || priorGpsSpeedAge > 10_000L) {
                resetFusionWithMeasurement(latestGpsFilteredKmh / 3.6f, gpsSpeedAccuracyMps)
            } else {
                correctFusionWithMeasurement(latestGpsFilteredKmh / 3.6f, gpsSpeedAccuracyMps)
            }

            lastValidSpeedElapsedMs = now
            lastSpeedEstimateElapsedMs = now
            latestSpeedEstimateReliable = true
            speedDisplayValid = true
            speedInvalidationLogged = false
        } else if (
            latestDerivedSpeedKmh.isFinite() &&
            latestDerivedSpeedKmh in 0f..250f &&
            latestAccuracy in 0f..40f
        ) {
            val derivedMps = latestDerivedSpeedKmh / 3.6f
            val derivedAccuracyMps = if (latestAccuracy <= 20f) 4.5f else 7.0f
            if (!fusionInitialized) {
                resetFusionWithMeasurement(derivedMps, derivedAccuracyMps)
            } else {
                correctFusionWithMeasurement(derivedMps, derivedAccuracyMps)
            }
            lastSpeedEstimateElapsedMs = now
            latestSpeedEstimateReliable = latestAccuracy <= 30f
            speedDisplayValid = latestSpeedEstimateReliable
            if (speedDisplayValid) speedInvalidationLogged = false
        } else {
            latestSpeedEstimateReliable = currentSpeedEstimateAgeMs() in 0L..3000L
            speedDisplayValid = latestSpeedEstimateReliable
        }

        // Strong stationary evidence must win over a slowly-decaying fusion estimate.
        // Two consecutive near-zero speed fixes are enough to snap the UI/fusion to 0.
        val rawStationary = speedValid && rawSpeed <= STATIONARY_RAW_SPEED_KMH
        val derivedStationary =
            !speedValid &&
                latestDerivedSpeedKmh.isFinite() &&
                latestDerivedSpeedKmh <= STATIONARY_DERIVED_SPEED_KMH &&
                latestAccuracy in 0f..15f

        if (rawStationary || derivedStationary) {
            stationarySpeedHits = min(STATIONARY_CONFIRM_HITS, stationarySpeedHits + 1)
        } else if (
            (speedValid && rawSpeed >= 5f) ||
            (latestDerivedSpeedKmh.isFinite() && latestDerivedSpeedKmh >= 6f)
        ) {
            stationarySpeedHits = 0
        }

        if (stationarySpeedHits >= STATIONARY_CONFIRM_HITS) {
            fusionSpeedMps = 0f
            fusionVariance = 0.5f
            fusionInitialized = true
            latestSpeedKmh = 0f
            latestSpeedEstimateReliable = true
            speedDisplayValid = true
            lastSpeedEstimateElapsedMs = now
            imuBridgeActive = false
            overspeedActive = false
        } else if (fusionInitialized) {
            latestSpeedKmh = (fusionSpeedMps * 3.6f).coerceIn(0f, 250f)
        } else if (latestDerivedSpeedKmh.isFinite()) {
            latestSpeedKmh = latestDerivedSpeedKmh.coerceIn(0f, 250f)
        } else if (speedValid) {
            latestSpeedKmh = rawSpeed.coerceIn(0f, 250f)
        }

        imuBridgeActive =
            stationarySpeedHits < STATIONARY_CONFIRM_HITS &&
                !speedValid &&
                currentSpeedEstimateAgeMs() in 0L..FUSION_BRIDGE_MS

        evaluateAllCameras()
        if (latestSpeedEstimateReliable) handleOverspeed(latestSpeedKmh)

        val speedReason = if (speedValid) "OK" else speedRejectionReason(location, speedAccuracyOk)
        appendRow(
            if (speedValid) "LOCATION" else "LOCATION_ONLY",
            Float.NaN, Float.NaN, Float.NaN, Float.NaN,
            0L,
            true,
            speedValid,
            "source=$source;speedReason=$speedReason;" +
                "derived=${if (latestDerivedSpeedKmh.isFinite()) String.format(Locale.US, "%.2f", latestDerivedSpeedKmh) else ""};" +
                "locationGapMs=$locationGap;" +
                "callbackGapMs=${if (previousCallback > 0L) now - previousCallback else -1L};" +
                "fusion=${if (fusionActive) "GPS_IMU" else "POSITION_OR_SPEED"}",
            null
        )

        recoveredLocationGapMs = 0L
        updatePrefs()
        updateNotification()
    }

    private fun locationRejectionReason(location: Location, ageMs: Long): String {
        return when {
            ageMs !in 0L..3000L -> "STALE_LOCATION"
            location.hasAccuracy() && location.accuracy > 50f -> "POOR_LOCATION_ACCURACY"
            !location.latitude.isFinite() || !location.longitude.isFinite() -> "NONFINITE_LOCATION"
            else -> "INVALID_LOCATION"
        }
    }

    private fun speedRejectionReason(location: Location, speedAccuracyOk: Boolean): String {
        val rawSpeed = if (location.hasSpeed()) location.speed * 3.6f else Float.NaN
        return when {
            !location.hasSpeed() -> "NO_SPEED"
            !speedAccuracyOk -> "POOR_SPEED_ACCURACY"
            rawSpeed !in 0f..250f -> "INVALID_SPEED"
            else -> "UNKNOWN_SPEED_REJECT"
        }
    }

    private fun locationAgeMs(location: Location): Long {
        return if (location.elapsedRealtimeNanos > 0L) {
            ((SystemClock.elapsedRealtimeNanos() - location.elapsedRealtimeNanos) / 1_000_000L)
                .coerceAtLeast(0L)
        } else {
            0L
        }
    }

    private fun filteredSpeed(raw: Float): Float {
        speedSamples.addLast(raw)
        while (speedSamples.size > 3) speedSamples.removeFirst()
        return speedSamples.sorted()[speedSamples.size / 2]
    }

    private fun updateDerivedSpeed(location: Location, now: Long): Float {
        val previous = previousPositionForSpeed
        val previousTime = previousPositionElapsedMs

        previousPositionForSpeed = Location(location)
        previousPositionElapsedMs = now

        if (previous == null || previousTime <= 0L) return Float.NaN
        val dt = (now - previousTime) / 1000f
        if (dt !in 0.4f..5.0f) {
            derivedSpeedSamples.clear()
            return Float.NaN
        }

        val prevAcc = if (previous.hasAccuracy()) previous.accuracy else 25f
        val curAcc = if (location.hasAccuracy()) location.accuracy else 25f
        if (prevAcc > 40f || curAcc > 40f) return Float.NaN

        val kmh = previous.distanceTo(location) / dt * 3.6f
        if (kmh !in 0f..250f) return Float.NaN

        derivedSpeedSamples.addLast(kmh)
        while (derivedSpeedSamples.size > 3) derivedSpeedSamples.removeFirst()
        return derivedSpeedSamples.sorted()[derivedSpeedSamples.size / 2]
    }

    private fun resetFusionWithMeasurement(speedMps: Float, accuracyMps: Float) {
        fusionSpeedMps = speedMps.coerceIn(0f, 70f)
        fusionVariance = max(1f, accuracyMps * accuracyMps)
        fusionInitialized = true
        fusionActive = linearAcceleration != null && rotationVector != null
        lastFusionSensorNs = 0L
    }

    private fun correctFusionWithMeasurement(speedMps: Float, accuracyMps: Float) {
        if (!fusionInitialized) {
            resetFusionWithMeasurement(speedMps, accuracyMps)
            return
        }

        val measurementVariance = max(0.25f, accuracyMps * accuracyMps)
        val gain = fusionVariance / (fusionVariance + measurementVariance)
        fusionSpeedMps += gain * (speedMps - fusionSpeedMps)
        fusionSpeedMps = fusionSpeedMps.coerceIn(0f, 70f)
        fusionVariance = max(0.05f, (1f - gain) * fusionVariance)
        fusionActive = linearAcceleration != null && rotationVector != null
    }

    private fun prepareForLocationGapRecovery(gapMs: Long) {
        speedSamples.clear()
        derivedSpeedSamples.clear()
        previousPositionForSpeed = null
        previousPositionElapsedMs = 0L

        // Never carry a pre-gap speed estimate into a recovered location. The old 0.10
        // behavior could keep 50-90 km/h on screen long after the car had stopped.
        fusionInitialized = false
        fusionActive = false
        fusionSpeedMps = 0f
        fusionVariance = 4f
        lastFusionSensorNs = 0L
        latestGpsFilteredKmh = Float.NaN
        latestDerivedSpeedKmh = Float.NaN
        latestSpeedKmh = 0f
        latestForwardAccelMps2 = 0f
        latestSpeedEstimateReliable = false
        speedDisplayValid = false
        imuBridgeActive = false
        stationarySpeedHits = 0
        overspeedActive = false
        lastValidSpeedElapsedMs = 0L
        lastSpeedEstimateElapsedMs = 0L
        speedInvalidationLogged = false

        for ((_, state) in cameraStates) {
            state.lastDistanceM = Float.MAX_VALUE
            state.awayHits = 0
            state.hardBadHits = 0
            if (state.phase == TrackPhase.TRACKING) {
                state.confirmHits = 0
                state.approachHits = 0
            }
        }

        if (loggingStarted) {
            appendMarker(
                "GPS_LOCATION_RECOVERED",
                "gapMs=$gapMs;satellites=$gnssSatellites;used=$gnssUsedInFix"
            )
        }
    }

    private fun predictFusion(forwardAccelMps2: Float, sensorTimestampNs: Long) {
        if (!fusionInitialized || !latestBearingValid || !rotationReady) {
            lastFusionSensorNs = sensorTimestampNs
            return
        }

        if (lastFusionSensorNs <= 0L) {
            lastFusionSensorNs = sensorTimestampNs
            return
        }

        val dt =
            ((sensorTimestampNs - lastFusionSensorNs) / 1_000_000_000.0f)
                .coerceIn(0f, 0.20f)
        lastFusionSensorNs = sensorTimestampNs
        if (dt <= 0f) return

        val estimateAge = currentSpeedEstimateAgeMs()
        if (estimateAge < 0L || estimateAge > FUSION_BRIDGE_MS) return

        val a =
            if (abs(forwardAccelMps2) < 0.08f) 0f
            else forwardAccelMps2.coerceIn(-8f, 8f)

        fusionSpeedMps = (fusionSpeedMps + a * dt).coerceIn(0f, 70f)
        fusionVariance = (fusionVariance + (0.7f * 0.7f * dt)).coerceAtMost(25f)
        latestForwardAccelMps2 = a
        latestSpeedKmh = (fusionSpeedMps * 3.6f).coerceIn(0f, 250f)
        imuBridgeActive = currentGpsSpeedAgeMs() > 1200L && estimateAge <= FUSION_BRIDGE_MS
        fusionActive = true

        if (!gpsStale && latestSpeedEstimateReliable) handleOverspeed(latestSpeedKmh)
        updatePrefs()
    }

    private fun updateForwardAcceleration(event: SensorEvent) {
        if (!rotationReady || !latestBearingValid || event.values.size < 3) return

        val x = event.values[0]
        val y = event.values[1]
        val z = event.values[2]

        val east =
            rotationMatrix[0] * x +
                rotationMatrix[1] * y +
                rotationMatrix[2] * z

        val north =
            rotationMatrix[3] * x +
                rotationMatrix[4] * y +
                rotationMatrix[5] * z

        val br = Math.toRadians(latestBearing.toDouble())
        val forward = (east * sin(br) + north * cos(br)).toFloat()
        predictFusion(forward, event.timestamp)
    }

    private fun updateLocationHealth() {
        val locationAge = currentGpsAgeMs()
        val staleNow = locationAge < 0L || locationAge > LOCATION_STALE_MS

        if (staleNow != gpsStale) {
            gpsStale = staleNow
            if (gpsStale && loggingStarted) {
                appendMarker(
                    "GPS_STALE",
                    "locationAgeMs=$locationAge;callbackAgeMs=${currentGpsCallbackAgeMs()};" +
                        "speedAgeMs=${currentGpsSpeedAgeMs()};satellites=$gnssSatellites;used=$gnssUsedInFix"
                )
            }
        }

        val estimateAge = currentSpeedEstimateAgeMs()

        // UI must never present an old numeric speed as if it were current. Keep the
        // internal estimate for a short bridge, but mark display speed invalid after 4 s.
        val displayFresh =
            !gpsStale &&
                estimateAge in 0L..SPEED_DISPLAY_STALE_MS &&
                latestSpeedEstimateReliable

        speedDisplayValid = displayFresh

        imuBridgeActive =
            fusionActive &&
                estimateAge in 1201..FUSION_BRIDGE_MS &&
                latestSpeedKmh >= 5f &&
                stationarySpeedHits < STATIONARY_CONFIRM_HITS

        if (
            (locationAge > SPEED_HARD_INVALIDATE_MS || estimateAge > SPEED_HARD_INVALIDATE_MS) &&
            (latestSpeedKmh > 0f || fusionInitialized)
        ) {
            if (!speedInvalidationLogged && loggingStarted) {
                appendMarker(
                    "SPEED_ESTIMATE_INVALIDATED",
                    "locationAgeMs=$locationAge;estimateAgeMs=$estimateAge;lastSpeed=${String.format(Locale.US, "%.1f", latestSpeedKmh)}"
                )
                speedInvalidationLogged = true
            }
            latestSpeedKmh = 0f
            latestSpeedEstimateReliable = false
            speedDisplayValid = false
            imuBridgeActive = false
            fusionInitialized = false
            fusionActive = false
            fusionSpeedMps = 0f
            latestForwardAccelMps2 = 0f
            overspeedActive = false
            stationarySpeedHits = 0
        }

        if (locationAge in GPS_CAMERA_BRIDGE_START_MS..GPS_CAMERA_BRIDGE_MAX_MS) {
            bridgeConfirmedCameraAlerts(locationAge)
        } else if (locationAge > GPS_CAMERA_BRIDGE_MAX_MS) {
            // Never leave a stale camera/limit on screen for minutes. This also prevents
            // a later generic speed beep from looking like a late camera alert.
            if (activeCameraId.isNotEmpty()) {
                clearUiCamera()
                cameraAlertZoneActive = false
            }
        }

        if (
            latestBearingValid &&
            lastBearingElapsedMs > 0L &&
            SystemClock.elapsedRealtime() - lastBearingElapsedMs > BEARING_HOLD_MS
        ) {
            latestBearingValid = false
        }

        updatePrefs()
    }

    private fun currentGpsAgeMs(): Long {
        if (lastValidLocationElapsedMs <= 0L) return -1L
        return (SystemClock.elapsedRealtime() - lastValidLocationElapsedMs).coerceAtLeast(0L)
    }

    private fun currentGpsSpeedAgeMs(): Long {
        if (lastValidSpeedElapsedMs <= 0L) return -1L
        return (SystemClock.elapsedRealtime() - lastValidSpeedElapsedMs).coerceAtLeast(0L)
    }

    private fun currentSpeedEstimateAgeMs(): Long {
        if (lastSpeedEstimateElapsedMs <= 0L) return -1L
        return (SystemClock.elapsedRealtime() - lastSpeedEstimateElapsedMs).coerceAtLeast(0L)
    }

    private fun currentGpsCallbackAgeMs(): Long {
        if (lastGpsCallbackElapsedMs <= 0L) return -1L
        return (SystemClock.elapsedRealtime() - lastGpsCallbackElapsedMs).coerceAtLeast(0L)
    }

    private fun evaluateAllCameras() {
        cameraAlertZoneActive = false

        if (
            latestLat.isNaN() ||
            latestLon.isNaN() ||
            cameraClusters.isEmpty() ||
            !latestBearingValid
        ) {
            if (confirmedCameraCount == 0) clearUiCamera()
            return
        }

        val now = SystemClock.elapsedRealtime()
        val from = Location("current").apply {
            latitude = latestLat
            longitude = latestLon
        }

        val confirmedGeometries = ArrayList<CameraGeometry>()
        val alertRequests = ArrayList<AlertRequest>()

        for (cluster in cameraClusters) {
            val camera = cluster.camera
            val to = Location("camera").apply {
                latitude = camera.latitude
                longitude = camera.longitude
            }

            val directDistance = from.distanceTo(to)
            if (directDistance > CAMERA_SEARCH_M) continue

            var state = cameraStates.getOrPut(cluster.key) { CameraTrackState() }

            if (
                (state.phase == TrackPhase.PASSED || state.phase == TrackPhase.ABANDONED) &&
                directDistance > 1200f &&
                now - state.transitionElapsed > 30_000L
            ) {
                state = CameraTrackState()
                cameraStates[cluster.key] = state
            }

            if (state.phase == TrackPhase.PASSED || state.phase == TrackPhase.ABANDONED) {
                state.lastSeenElapsed = now
                continue
            }

            val geometry = buildCameraGeometry(from, to, cluster, state, now)
            state.lastSeenElapsed = now
            state.minDistanceM = min(state.minDistanceM, geometry.directDistanceM)
            updateApproachState(state, geometry)

            val withinConfirmRange =
                geometry.directDistanceM <=
                    max(950f, geometry.warningDistanceM + CAMERA_CONFIRM_EXTRA_M)

            val strongApproachHistory = state.approachHits >= 2
            val nearFastTrack =
                geometry.directDistanceM <= 500f &&
                    strongApproachHistory &&
                    geometry.hardGeometryOk &&
                    geometry.matchScore >= geometry.requiredScore - 1

            val confirmEligible =
                withinConfirmRange &&
                    geometry.hardGeometryOk &&
                    (
                        geometry.matchScore >= geometry.requiredScore ||
                            nearFastTrack
                    ) &&
                    (
                        state.approachHits >= 1 ||
                            geometry.directDistanceM <= 250f
                    )

            if (state.phase == TrackPhase.TRACKING) {
                if (confirmEligible) {
                    state.confirmHits++
                    val hitsNeeded = if (nearFastTrack) 1 else geometry.requiredHits
                    if (state.confirmHits >= hitsNeeded) {
                        state.phase = TrackPhase.CONFIRMED
                        state.transitionElapsed = now
                        state.hardBadHits = 0
                        appendCameraEvent(
                            "CAMERA_MATCH_CONFIRMED",
                            geometry,
                            state,
                            "confirmHits=${state.confirmHits};hitsNeeded=$hitsNeeded;" +
                                "approachHits=${state.approachHits};fastTrack=$nearFastTrack"
                        )
                    }
                } else {
                    state.confirmHits = max(0, state.confirmHits - 1)
                }
            }

            if (
                state.phase == TrackPhase.TRACKING &&
                geometry.directDistanceM <= CAMERA_DIAG_NEAR_M &&
                now - state.lastDiagElapsed >= CAMERA_DIAG_INTERVAL_MS
            ) {
                state.lastDiagElapsed = now
                appendCameraEvent(
                    "CAMERA_MATCH_DIAG",
                    geometry,
                    state,
                    "reason=${geometry.matchReason};confirmHits=${state.confirmHits};" +
                        "approachHits=${state.approachHits};awayHits=${state.awayHits}"
                )
            }

            if (
                geometry.directDistanceM <= NEAR_PASS_AUDIT_M &&
                !state.nearPassAudited
            ) {
                state.nearPassAudited = true
                appendCameraEvent(
                    "CAMERA_NEAR_PASS_AUDIT",
                    geometry,
                    state,
                    "phase=${state.phase};warned=${state.warned}"
                )
            }

            if (state.phase == TrackPhase.CONFIRMED) {
                state.bridgeLogged = false
                state.lastGeometry = geometry
                state.lastGeometryElapsed = now

                if (geometry.hardGeometryOk) state.hardBadHits = 0 else state.hardBadHits++

                // If a long location gap ended after the camera had already moved behind us,
                // never emit a late beep. Record the miss explicitly instead.
                if (
                    recoveredLocationGapMs > LOCATION_STALE_MS &&
                    geometry.vehicleForwardM <= 0f
                ) {
                    if (!state.warned && !state.missedDuringGapLogged) {
                        state.missedDuringGapLogged = true
                        appendCameraEvent(
                            "CAMERA_MISSED_DURING_GPS_GAP",
                            geometry,
                            state,
                            "gapMs=$recoveredLocationGapMs;minDistance=${String.format(Locale.US, "%.1f", state.minDistanceM)}"
                        )
                    }
                    markCameraPassed(geometry, state, now)
                    state.lastDistanceM = geometry.directDistanceM
                    continue
                }

                val clearlyBehind = geometry.forwardDeltaDeg >= PASS_BEHIND_DEG
                val distanceGrowing =
                    state.lastDistanceM != Float.MAX_VALUE &&
                        geometry.directDistanceM > state.minDistanceM + PASS_DISTANCE_GROWTH_M

                if (clearlyBehind && distanceGrowing) {
                    state.awayHits++
                    if (state.awayHits >= 2) {
                        markCameraPassed(geometry, state, now)
                        state.lastDistanceM = geometry.directDistanceM
                        continue
                    }
                }

                if (
                    state.hardBadHits >= 4 &&
                    state.awayHits >= 3 &&
                    geometry.directDistanceM > 250f
                ) {
                    state.phase = TrackPhase.ABANDONED
                    state.transitionElapsed = now
                    state.alertLevel = 0
                    appendCameraEvent(
                        "CAMERA_ABANDONED",
                        geometry,
                        state,
                        "reason=hard_geometry_failed_while_moving_away;hardBadHits=${state.hardBadHits}"
                    )
                    state.lastDistanceM = geometry.directDistanceM
                    continue
                }

                if (geometry.vehicleForwardM > 0f && geometry.forwardDeltaDeg < PASS_BEHIND_DEG) {
                    confirmedGeometries += geometry
                }

                computeAlertRequest(geometry, state, false)?.let { alertRequests += it }
            }

            state.lastDistanceM = geometry.directDistanceM
        }

        confirmedCameraCount = confirmedGeometries.count { it.vehicleForwardM > 0f }
        chooseUiCamera(confirmedGeometries)

        cameraAlertZoneActive = alertRequests.isNotEmpty()
        dispatchCameraAlerts(alertRequests)
        cleanupOldCameraStates(now)
    }

    private fun markCameraPassed(
        geometry: CameraGeometry,
        state: CameraTrackState,
        now: Long
    ) {
        appendCameraEvent(
            "CAMERA_PASSED",
            geometry,
            state,
            "warned=${state.warned};minDistance=${String.format(Locale.US, "%.1f", state.minDistanceM)}"
        )

        if (!state.warned && state.minDistanceM <= NEAR_PASS_AUDIT_M) {
            appendCameraEvent(
                "CAMERA_MISSED_DIAGNOSTIC",
                geometry,
                state,
                "physically_near_camera_but_no_warning=true"
            )
        }

        state.phase = TrackPhase.PASSED
        state.transitionElapsed = now
        state.alertLevel = 0
    }

    private fun buildCameraGeometry(
        from: Location,
        to: Location,
        cluster: CameraCluster,
        state: CameraTrackState,
        now: Long
    ): CameraGeometry {
        val camera = cluster.camera
        val distance = from.distanceTo(to)
        val bearingToCamera = normalizeBearing(from.bearingTo(to))
        val forwardDelta = angleDifference(latestBearing, bearingToCamera)

        val vehicleForward =
            (distance * cos(Math.toRadians(forwardDelta.toDouble()))).toFloat()
        val vehicleCross =
            abs(distance * sin(Math.toRadians(forwardDelta.toDouble()))).toFloat()

        val rb = camera.roadBearing
        val roadAxisDelta =
            if (rb != null) {
                min(
                    angleDifference(latestBearing, normalizeBearing(rb)),
                    angleDifference(latestBearing, normalizeBearing(rb + 180f))
                )
            } else {
                0f
            }

        val roadDirectionDelta =
            if (rb != null) {
                if (hasReliableDirectedBearing(camera)) {
                    angleDifference(latestBearing, normalizeBearing(rb))
                } else {
                    roadAxisDelta
                }
            } else {
                0f
            }

        val roadCross =
            if (rb != null) {
                val cameraAxisDelta = min(
                    angleDifference(normalizeBearing(rb), bearingToCamera),
                    angleDifference(normalizeBearing(rb + 180f), bearingToCamera)
                )
                abs(
                    distance * sin(Math.toRadians(cameraAxisDelta.toDouble()))
                ).toFloat()
            } else {
                vehicleCross
            }

        // Curve-aware cross-track: a curved road can make the camera-road tangent and
        // the car's instantaneous heading disagree strongly at long range. We hard-reject
        // only when BOTH independent cross-track views are bad. Score can never override it.
        val curveCross = if (rb != null) min(vehicleCross, roadCross) else vehicleCross

        val baseHardCrossLimit =
            when {
                distance > 1500f -> 230f
                distance > 1100f -> 210f
                distance > 800f -> 195f
                distance > 600f -> 178f
                distance > 450f -> 145f
                distance > 300f -> 115f
                else -> 80f
            }

        val accuracyAllowance =
            if (latestAccuracy >= 0f) (latestAccuracy * 1.2f).coerceIn(0f, 20f) else 10f
        val hardCrossLimit = baseHardCrossLimit + accuracyAllowance
        val hardCrossOk = curveCross <= hardCrossLimit

        val forwardLimit =
            when {
                distance > 1100f -> 45f
                distance > 750f -> 50f
                distance > 550f -> 55f
                distance > 350f -> 68f
                else -> 85f
            }

        val roadDirectionLimit =
            when {
                distance > 1100f -> 42f
                distance > 750f -> 48f
                distance > 550f -> 55f
                distance > 350f -> 65f
                else -> 78f
            }

        val directionGateOk =
            forwardDelta <= forwardLimit &&
                (
                    rb == null ||
                        roadDirectionDelta <= roadDirectionLimit
                    )

        val hardGeometryOk =
            vehicleForward > 8f &&
                hardCrossOk &&
                directionGateOk

        val candidateLimit = if (camera.type == "speed") validDynamicLimit(camera) else null
        val warningDistance = computeAdaptiveWarningDistanceM(
            latestSpeedKmh,
            candidateLimit,
            camera.type
        )

        val speedMps = max(1.5f, latestSpeedKmh / 3.6f)
        val ttc = if (vehicleForward > 0f) vehicleForward / speedMps else -1f

        val dtS =
            if (state.lastSeenElapsed > 0L) {
                (now - state.lastSeenElapsed).coerceAtLeast(1L) / 1000f
            } else {
                0f
            }

        val approachDelta =
            if (state.lastDistanceM != Float.MAX_VALUE) state.lastDistanceM - distance else 0f
        val approachRate = if (dtS > 0f) approachDelta / dtS else 0f

        var score = 0
        val reasons = ArrayList<String>()

        if (!hardCrossOk) {
            reasons += "hard_curve_cross"
        } else {
            when {
                curveCross <= hardCrossLimit * 0.35f -> score += 3
                curveCross <= hardCrossLimit * 0.65f -> score += 2
                else -> score += 1
            }
        }

        when {
            forwardDelta <= 20f -> score += 3
            forwardDelta <= 35f -> score += 2
            forwardDelta <= 55f -> score += 1
            else -> reasons += if (forwardDelta >= PASS_BEHIND_DEG) "behind_vehicle" else "forward_angle"
        }

        if (rb != null) {
            when {
                roadDirectionDelta <= 20f -> score += 2
                roadDirectionDelta <= 40f -> score += 1
                else -> reasons += if (hasReliableDirectedBearing(camera)) "road_direction" else "road_axis"
            }
        } else {
            score += 1
        }

        if (vehicleForward > 8f) score += 1 else reasons += "not_ahead"

        if (approachDelta > 2f) {
            score += 2
        } else if (approachDelta < -3f) {
            reasons += "moving_away"
        }

        when {
            latestAccuracy in 0f..8f -> score += 2
            latestAccuracy in 0f..18f -> score += 1
        }

        if (!directionGateOk && forwardDelta < PASS_BEHIND_DEG) {
            reasons += "direction_gate"
        }

        val requiredScore =
            when {
                distance > 1200f -> 9
                distance > 850f -> 8
                distance > 550f -> 7
                distance > 320f -> 6
                else -> 5
            }

        val requiredHits =
            when {
                distance > 850f -> 3
                distance > 600f -> 3
                distance > 350f -> 2
                else -> 1
            }

        return CameraGeometry(
            cluster = cluster,
            directDistanceM = distance,
            vehicleForwardM = vehicleForward,
            vehicleCrossM = vehicleCross,
            bearingToCameraDeg = bearingToCamera,
            forwardDeltaDeg = forwardDelta,
            roadCrossM = roadCross,
            curveCrossM = curveCross,
            roadAxisDeltaDeg = roadAxisDelta,
            roadDirectionDeltaDeg = roadDirectionDelta,
            hardCrossLimitM = hardCrossLimit,
            hardCrossOk = hardCrossOk,
            directionGateOk = directionGateOk,
            hardGeometryOk = hardGeometryOk,
            warningDistanceM = warningDistance,
            ttcS = ttc,
            candidateLimit = candidateLimit,
            approachDeltaM = approachDelta,
            approachRateMps = approachRate,
            matchScore = score,
            requiredScore = requiredScore,
            requiredHits = requiredHits,
            matchReason = if (reasons.isEmpty()) "good" else reasons.distinct().joinToString("+")
        )
    }

    private fun updateApproachState(state: CameraTrackState, geometry: CameraGeometry) {
        if (state.lastDistanceM == Float.MAX_VALUE) return
        when {
            geometry.approachDeltaM > 2f -> {
                state.approachHits = min(10, state.approachHits + 1)
                state.awayHits = max(0, state.awayHits - 1)
            }
            geometry.approachDeltaM < -3f -> {
                state.awayHits = min(10, state.awayHits + 1)
                state.approachHits = max(0, state.approachHits - 1)
            }
        }
    }

    private fun computeAlertRequest(
        geometry: CameraGeometry,
        state: CameraTrackState,
        bridge: Boolean
    ): AlertRequest? {
        if (state.phase != TrackPhase.CONFIRMED) return null
        if (geometry.vehicleForwardM <= 0f || geometry.forwardDeltaDeg >= PASS_BEHIND_DEG) {
            state.alertLevel = 0
            return null
        }

        // Hard geometry is never rescued by score. The cross-track itself is now curve-aware.
        if (!geometry.hardGeometryOk) {
            state.alertLevel = 0
            return null
        }

        val inWarningZone = geometry.vehicleForwardM <= geometry.warningDistanceM + 90f
        if (!inWarningZone) {
            state.alertLevel = 0
            return null
        }

        val level = computeCameraAlertLevel(
            geometry.vehicleForwardM,
            geometry.ttcS,
            latestSpeedKmh,
            geometry.candidateLimit,
            geometry.warningDistanceM
        )
        state.alertLevel = level

        if (!state.warned) {
            state.warned = true
            state.warningStartedElapsed = SystemClock.elapsedRealtime()
            appendCameraEvent(
                "CAMERA_WARNING",
                geometry,
                state,
                "first_warning=true;level=$level;bridge=$bridge"
            )
        }

        if (level >= 3 && !state.urgentLogged) {
            state.urgentLogged = true
            appendCameraEvent(
                "CAMERA_URGENT",
                geometry,
                state,
                "level=$level;bridge=$bridge"
            )
        }

        return AlertRequest(geometry, state, level, bridge)
    }

    private fun dispatchCameraAlerts(requests: List<AlertRequest>) {
        if (requests.isEmpty()) return
        val now = SystemClock.elapsedRealtime()

        val sorted = requests.sortedWith(
            compareByDescending<AlertRequest> { it.level }
                .thenBy { if (it.geometry.ttcS > 0f) it.geometry.ttcS else Float.MAX_VALUE }
                .thenBy { it.geometry.vehicleForwardM }
        )

        // Guarantee the FIRST audible alert for every confirmed physical cluster.
        // Arbitration may delay it briefly to avoid overlapping tones, but can no longer
        // suppress it for tens of seconds as happened in 0.9.
        val pendingFirst = sorted.filter { !it.state.firstAlertDelivered }
        if (pendingFirst.isNotEmpty()) {
            val chosen = pendingFirst.first()
            val pendingAge = now - chosen.state.warningStartedElapsed
            val globalGap = now - globalLastCameraPulseElapsed
            if (globalGap >= GLOBAL_ALERT_MIN_GAP_MS || pendingAge >= FIRST_ALERT_FORCE_MS) {
                globalLastCameraPulseElapsed = now
                globalLastCameraKey = chosen.geometry.cluster.key
                chosen.state.firstAlertDelivered = true
                chosen.state.lastPulseElapsed = now
                dispatchCameraAlertOutput(
                    chosen,
                    max(0, requests.size - 1),
                    true
                )
                return
            }
        }

        val repeatCandidates = sorted.filter {
            val interval = alertRepeatIntervalMs(it.level)
            now - it.state.lastPulseElapsed >= interval
        }
        if (repeatCandidates.isEmpty()) return
        if (now - globalLastCameraPulseElapsed < GLOBAL_ALERT_MIN_GAP_MS) return

        val chosen = repeatCandidates.first()
        globalLastCameraPulseElapsed = now
        globalLastCameraKey = chosen.geometry.cluster.key
        chosen.state.lastPulseElapsed = now
        dispatchCameraAlertOutput(
            chosen,
            max(0, requests.size - 1),
            false
        )
    }

    private fun alertRepeatIntervalMs(level: Int): Long =
        when (level) {
            1 -> 5000L
            2 -> 3000L
            3 -> 1500L
            4 -> 900L
            else -> Long.MAX_VALUE
        }

    private fun bridgeConfirmedCameraAlerts(locationAgeMs: Long) {
        if (
            locationAgeMs < GPS_CAMERA_BRIDGE_START_MS ||
            locationAgeMs > GPS_CAMERA_BRIDGE_MAX_MS
        ) {
            return
        }

        val now = SystemClock.elapsedRealtime()
        val speedMps = max(0f, latestSpeedKmh / 3.6f)
        if (speedMps < 1.0f) return

        val requests = ArrayList<AlertRequest>()

        for ((_, state) in cameraStates) {
            if (state.phase != TrackPhase.CONFIRMED) continue
            val last = state.lastGeometry ?: continue
            if (!last.hardGeometryOk) continue

            val elapsedS =
                (now - state.lastGeometryElapsed).coerceAtLeast(0L) / 1000f
            if (elapsedS <= 0f || elapsedS > GPS_CAMERA_BRIDGE_MAX_MS / 1000f) continue

            val predictedForward = last.vehicleForwardM - speedMps * elapsedS
            if (predictedForward <= 0f) {
                if (!state.warned && !state.missedDuringGapLogged) {
                    state.missedDuringGapLogged = true
                    appendCameraEvent(
                        "CAMERA_PREDICTED_PASS_DURING_GPS_GAP",
                        last,
                        state,
                        "locationAgeMs=$locationAgeMs;elapsedS=${String.format(Locale.US, "%.1f", elapsedS)}"
                    )
                }
                continue
            }

            val predictedDirect = max(0f, last.directDistanceM - speedMps * elapsedS)
            val predictedTtc = predictedForward / max(1.5f, speedMps)
            val bridged = last.copy(
                directDistanceM = predictedDirect,
                vehicleForwardM = predictedForward,
                ttcS = predictedTtc,
                approachDeltaM = 0f,
                approachRateMps = speedMps
            )

            if (!state.bridgeLogged) {
                state.bridgeLogged = true
                appendCameraEvent(
                    "CAMERA_GPS_BRIDGE_START",
                    bridged,
                    state,
                    "locationAgeMs=$locationAgeMs;maxBridgeMs=$GPS_CAMERA_BRIDGE_MAX_MS"
                )
            }

            computeAlertRequest(bridged, state, true)?.let { requests += it }
        }

        if (requests.isNotEmpty()) {
            cameraAlertZoneActive = true
            dispatchCameraAlerts(requests)
        }
    }

    private fun chooseUiCamera(geometries: List<CameraGeometry>) {
        val ahead = geometries.filter {
            it.vehicleForwardM > 0f &&
                it.forwardDeltaDeg < PASS_BEHIND_DEG &&
                it.hardGeometryOk
        }

        if (ahead.isEmpty()) {
            clearUiCamera()
            return
        }

        val best = ahead.minByOrNull {
            if (it.ttcS > 0f) it.ttcS else Float.MAX_VALUE
        } ?: run {
            clearUiCamera()
            return
        }

        val state = cameraStates[best.cluster.key]
        activeCameraId =
            if (best.cluster.memberIds.size > 1) {
                best.cluster.camera.id + "(+${best.cluster.memberIds.size - 1})"
            } else {
                best.cluster.camera.id
            }
        activeCameraType = best.cluster.camera.type
        activeCameraDistanceM = best.vehicleForwardM
        activeCameraSpeedLimit = best.candidateLimit
        activeCameraRoadName = best.cluster.camera.roadName
        activeCameraCurveCrossM = best.curveCrossM
        activeCameraForwardDelta = best.forwardDeltaDeg
        activeCameraAlertLevel = state?.alertLevel ?: 0
        activeCameraTtcS = best.ttcS
        activeCameraWarningDistanceM = best.warningDistanceM

        activeLimitKmh =
            if (
                best.vehicleForwardM <= best.warningDistanceM + 90f &&
                best.candidateLimit != null
            ) {
                best.candidateLimit
            } else {
                manualThresholdKmh
            }
    }

    private fun clearUiCamera() {
        activeCameraId = ""
        activeCameraType = ""
        activeCameraDistanceM = -1f
        activeCameraSpeedLimit = null
        activeCameraRoadName = ""
        activeCameraCurveCrossM = -1f
        activeCameraForwardDelta = -1f
        activeCameraAlertLevel = 0
        activeCameraTtcS = -1f
        activeCameraWarningDistanceM = -1f
        confirmedCameraCount = 0
        activeLimitKmh = manualThresholdKmh
    }

    private fun cleanupOldCameraStates(now: Long) {
        val iterator = cameraStates.iterator()
        while (iterator.hasNext()) {
            val entry = iterator.next()
            if (now - entry.value.lastSeenElapsed > 180_000L) iterator.remove()
        }
    }

    private fun computeAdaptiveWarningDistanceM(
        speedKmh: Float,
        cameraLimitKmh: Int?,
        cameraType: String
    ): Float {
        val v = max(5f, speedKmh) / 3.6f
        val baseLeadS =
            (24f + (speedKmh - 45f).coerceAtLeast(0f) * 0.12f)
                .coerceIn(24f, 32f)

        var warningM = v * baseLeadS

        if (cameraLimitKmh != null && speedKmh > cameraLimitKmh) {
            val target = cameraLimitKmh / 3.6f
            val decelDistance =
                ((v * v - target * target) / (2f * 1.6f)).coerceAtLeast(0f)
            val reactionDistance = v * 5f
            warningM = max(warningM, decelDistance + reactionDistance + 90f)
        }

        if (cameraType == "red_light") {
            warningM = max(warningM, v * 26f)
        }

        return warningM.coerceIn(440f, 1250f)
    }

    private fun computeCameraAlertLevel(
        forwardM: Float,
        ttcS: Float,
        speedKmh: Float,
        cameraLimitKmh: Int?,
        warningDistanceM: Float
    ): Int {
        val overLimit = cameraLimitKmh != null && speedKmh > cameraLimitKmh + 2f
        return when {
            (ttcS in 0f..6f || forwardM <= 140f) && overLimit -> 4
            ttcS in 0f..9f || forwardM <= 220f -> 3
            ttcS in 0f..15f || forwardM <= min(420f, warningDistanceM * 0.72f) -> 2
            else -> 1
        }
    }

    private fun dispatchCameraAlertOutput(
        request: AlertRequest,
        suppressedCount: Int,
        firstAlert: Boolean
    ) {
        val geometry = request.geometry
        val level = request.level
        val tone = when (level) {
            1 -> toneLow
            2 -> toneMid
            else -> toneHigh
        }
        val duration = when (level) {
            1 -> 120
            2 -> 170
            3 -> 240
            else -> 360
        }
        val toneId =
            if (geometry.cluster.camera.type == "red_light") {
                ToneGenerator.TONE_PROP_ACK
            } else {
                ToneGenerator.TONE_PROP_BEEP2
            }

        val toneOk = try {
            tone?.startTone(toneId, duration) ?: false
        } catch (_: Exception) {
            false
        }

        val vibrator = getSystemService(Context.VIBRATOR_SERVICE) as Vibrator
        val vibratorPresent = try { vibrator.hasVibrator() } catch (_: Exception) { false }
        var vibrationDispatched = false

        if (vibratorPresent) {
            try {
                when (level) {
                    1 -> vibratePattern(longArrayOf(0, 70), intArrayOf(0, 120))
                    2 -> vibratePattern(longArrayOf(0, 110), intArrayOf(0, 180))
                    3 -> vibratePattern(longArrayOf(0, 140), intArrayOf(0, 230))
                    else -> vibratePattern(
                        longArrayOf(0, 200, 70, 200),
                        intArrayOf(0, 255, 0, 255)
                    )
                }
                vibrationDispatched = true
            } catch (_: Exception) {}
        }

        val alarmVolume = try {
            audioManager.getStreamVolume(AudioManager.STREAM_ALARM)
        } catch (_: Exception) { -1 }
        val alarmMax = try {
            audioManager.getStreamMaxVolume(AudioManager.STREAM_ALARM)
        } catch (_: Exception) { -1 }

        appendCameraEvent(
            "ALERT_OUTPUT",
            geometry,
            request.state,
            "level=$level;bridge=${request.bridge};first_alert=$firstAlert;" +
                "tone_start_ok=$toneOk;vibrator_present=$vibratorPresent;" +
                "vibration_dispatched=$vibrationDispatched;alarm_volume=$alarmVolume;" +
                "alarm_max=$alarmMax;suppressed_other_camera_alerts=$suppressedCount;" +
                "stream=ALARM;duration_ms=$duration"
        )
    }

    private fun validDynamicLimit(camera: CameraPoint): Int? {
        val limit = camera.speedLimit ?: return null
        val confidence = camera.speedConfidence.lowercase(Locale.US)
        if (limit !in 20..150) return null
        if (confidence != "high" && confidence != "medium") return null
        if ((camera.roadDistanceM ?: 999.0) > 30.0) return null
        return limit
    }

    private fun appendCameraEvent(
        event: String,
        geometry: CameraGeometry,
        state: CameraTrackState,
        extra: String
    ) {
        appendRow(
            event,
            Float.NaN, Float.NaN, Float.NaN, Float.NaN,
            0L,
            !gpsStale,
            latestSpeedEstimateReliable,
            "cluster=${geometry.cluster.key};" +
                "memberIds=${geometry.cluster.memberIds.joinToString("|")};" +
                "direct=${String.format(Locale.US, "%.1f", geometry.directDistanceM)};" +
                "forward=${String.format(Locale.US, "%.1f", geometry.vehicleForwardM)};" +
                "vehicleCross=${String.format(Locale.US, "%.1f", geometry.vehicleCrossM)};" +
                "roadCross=${String.format(Locale.US, "%.1f", geometry.roadCrossM)};" +
                "curveCross=${String.format(Locale.US, "%.1f", geometry.curveCrossM)};" +
                "hardCrossLimit=${String.format(Locale.US, "%.1f", geometry.hardCrossLimitM)};" +
                "hardCrossOk=${geometry.hardCrossOk};directionGateOk=${geometry.directionGateOk};" +
                "hardGeometryOk=${geometry.hardGeometryOk};" +
                "bearingTo=${String.format(Locale.US, "%.1f", geometry.bearingToCameraDeg)};" +
                "forwardDelta=${String.format(Locale.US, "%.1f", geometry.forwardDeltaDeg)};" +
                "axisDelta=${String.format(Locale.US, "%.1f", geometry.roadAxisDeltaDeg)};" +
                "directionDelta=${String.format(Locale.US, "%.1f", geometry.roadDirectionDeltaDeg)};" +
                "approachDelta=${String.format(Locale.US, "%.1f", geometry.approachDeltaM)};" +
                "approachRate=${String.format(Locale.US, "%.1f", geometry.approachRateMps)};" +
                "score=${geometry.matchScore};requiredScore=${geometry.requiredScore};" +
                "requiredHits=${geometry.requiredHits};phase=${state.phase};warned=${state.warned};" +
                extra,
            geometry
        )
    }

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

    private fun handleOverspeed(speedKmh: Float) {
        if (gpsStale || !latestSpeedEstimateReliable) return

        if (abs(activeLimitKmh - lastAlertLimitKmh) >= 3) {
            overspeedActive = false
            lastAlertLimitKmh = activeLimitKmh
        }

        val now = SystemClock.elapsedRealtime()
        if (speedKmh >= activeLimitKmh + 1f) {
            if (!overspeedActive || now - lastOverspeedAlertElapsed >= OVERSPEED_REPEAT_MS) {
                overspeedActive = true
                lastOverspeedAlertElapsed = now

                if (!cameraAlertZoneActive) overspeedAlert()
                appendMarker(
                    "OVERSPEED_ALERT",
                    "speed=${speedKmh.roundToInt()};limit=$activeLimitKmh;camera=$activeCameraId"
                )
            }
        } else if (speedKmh <= activeLimitKmh - 4f) {
            overspeedActive = false
        }
    }

    private fun overspeedAlert() {
        val toneOk = try {
            toneLow?.startTone(ToneGenerator.TONE_PROP_ACK, 110) ?: false
        } catch (_: Exception) {
            false
        }

        val vibrator = getSystemService(Context.VIBRATOR_SERVICE) as Vibrator
        val hasVibrator = try { vibrator.hasVibrator() } catch (_: Exception) { false }
        var vibrationDispatched = false

        if (hasVibrator) {
            try {
                vibratePattern(
                    longArrayOf(0, 65),
                    intArrayOf(0, 90)
                )
                vibrationDispatched = true
            } catch (_: Exception) {}
        }

        val alarmVolume = try {
            audioManager.getStreamVolume(AudioManager.STREAM_ALARM)
        } catch (_: Exception) { -1 }
        val alarmMax = try {
            audioManager.getStreamMaxVolume(AudioManager.STREAM_ALARM)
        } catch (_: Exception) { -1 }

        appendMarker(
            "OVERSPEED_OUTPUT",
            "alert_kind=OVERSPEED_SOFT;tone_start_ok=$toneOk;vibrator_present=$hasVibrator;" +
                "vibration_dispatched=$vibrationDispatched;alarm_volume=$alarmVolume;alarm_max=$alarmMax"
        )
    }

    private fun vibratePattern(timings: LongArray, amplitudes: IntArray) {
        val vibrator = getSystemService(Context.VIBRATOR_SERVICE) as Vibrator
        if (Build.VERSION.SDK_INT >= 26) {
            vibrator.vibrate(VibrationEffect.createWaveform(timings, amplitudes, -1))
        } else {
            @Suppress("DEPRECATION")
            vibrator.vibrate(timings, -1)
        }
    }

    override fun onSensorChanged(event: SensorEvent) {
        if (event.values.size < 3) return

        when (event.sensor.type) {
            Sensor.TYPE_ROTATION_VECTOR -> {
                if (
                    lastRotationProcessNs > 0L &&
                    event.timestamp - lastRotationProcessNs < SENSOR_PROCESS_PERIOD_NS
                ) return

                lastRotationProcessNs = event.timestamp
                rotationProcessedCount++
                try {
                    SensorManager.getRotationMatrixFromVector(rotationMatrix, event.values)
                    rotationReady = true
                } catch (_: Exception) {
                    rotationReady = false
                }
            }

            Sensor.TYPE_LINEAR_ACCELERATION -> {
                if (
                    lastLinearProcessNs > 0L &&
                    event.timestamp - lastLinearProcessNs < SENSOR_PROCESS_PERIOD_NS
                ) return

                lastLinearProcessNs = event.timestamp
                linearProcessedCount++
                updateForwardAcceleration(event)
            }

            Sensor.TYPE_GYROSCOPE -> {
                if (
                    lastGyroProcessNs > 0L &&
                    event.timestamp - lastGyroProcessNs < SENSOR_PROCESS_PERIOD_NS
                ) return

                lastGyroProcessNs = event.timestamp
                gyroProcessedCount++
                val gx = event.values[0]
                val gy = event.values[1]
                val gz = event.values[2]
                latestGyroMagnitude = sqrt((gx * gx + gy * gy + gz * gz).toDouble()).toFloat()
            }

            Sensor.TYPE_ACCELEROMETER -> {
                if (
                    lastAccelProcessNs > 0L &&
                    event.timestamp - lastAccelProcessNs < SENSOR_PROCESS_PERIOD_NS
                ) return

                lastAccelProcessNs = event.timestamp
                accelProcessedCount++
                if (!loggingStarted) return

                val x = event.values[0]
                val y = event.values[1]
                val z = event.values[2]
                val magnitude = sqrt((x * x + y * y + z * z).toDouble()).toFloat()
                val shock = abs(magnitude - SensorManager.GRAVITY_EARTH)
                val now = SystemClock.elapsedRealtime()

                if (
                    speedDisplayValid &&
                    latestSpeedKmh >= 8f &&
                    shock >= ROAD_IMPACT_SHOCK_MPS2 &&
                    now - lastRoadImpactElapsed >= ROAD_IMPACT_COOLDOWN_MS
                ) {
                    lastRoadImpactElapsed = now
                    val severity = when {
                        shock >= 7.0f -> "severe"
                        shock >= 5.0f -> "strong"
                        shock >= 4.0f -> "medium"
                        else -> "light"
                    }

                    val locationFresh = currentGpsAgeMs() in 0L..3000L
                    appendRow(
                        if (locationFresh) "ROAD_IMPACT_CANDIDATE" else "ROAD_IMPACT_UNLOCATED",
                        x, y, z, magnitude,
                        event.timestamp,
                        locationFresh,
                        latestSpeedEstimateReliable,
                        "shock=${String.format(Locale.US, "%.2f", shock)};" +
                            "gyroMag=${String.format(Locale.US, "%.3f", latestGyroMagnitude)};" +
                            "speed=${String.format(Locale.US, "%.1f", latestSpeedKmh)};" +
                            "severity=$severity;locationFresh=$locationFresh",
                        null,
                        includePosition = locationFresh
                    )
                }
            }
        }
    }

    private fun maybeLogSensorHealth() {
        if (!loggingStarted) return

        val now = SystemClock.elapsedRealtime()
        if (lastSensorHealthElapsed == 0L) {
            lastSensorHealthElapsed = now
            return
        }

        val elapsed = now - lastSensorHealthElapsed
        if (elapsed < SENSOR_HEALTH_LOG_MS) return

        val seconds = elapsed / 1000f
        val accelHz = accelProcessedCount / max(0.1f, seconds)
        val gyroHz = gyroProcessedCount / max(0.1f, seconds)
        val linearHz = linearProcessedCount / max(0.1f, seconds)
        val rotationHz = rotationProcessedCount / max(0.1f, seconds)
        val gnssStatusAge =
            if (lastGnssStatusElapsedMs > 0L) now - lastGnssStatusElapsedMs else -1L

        appendMarker(
            "SENSOR_HEALTH",
            "accelHz=${String.format(Locale.US, "%.1f", accelHz)};" +
                "gyroHz=${String.format(Locale.US, "%.1f", gyroHz)};" +
                "linearHz=${String.format(Locale.US, "%.1f", linearHz)};" +
                "rotationHz=${String.format(Locale.US, "%.1f", rotationHz)};" +
                "rawSensorCsv=false;locationAgeMs=${currentGpsAgeMs()};" +
                "speedAgeMs=${currentGpsSpeedAgeMs()};callbackAgeMs=${currentGpsCallbackAgeMs()};" +
                "satellites=$gnssSatellites;used=$gnssUsedInFix;gnssStatusAgeMs=$gnssStatusAge;" +
                "provider=$latestProvider;fused=$fusedUpdatesRegistered;legacy=$legacyLocationUpdatesRegistered;" +
                "stallCount=$locationStallCount"
        )

        accelProcessedCount = 0
        gyroProcessedCount = 0
        linearProcessedCount = 0
        rotationProcessedCount = 0
        lastSensorHealthElapsed = now
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) = Unit

    @Synchronized
    private fun appendMarker(marker: String, details: String = "") {
        if (!::logFile.isInitialized) return
        appendRow(
            marker,
            Float.NaN, Float.NaN, Float.NaN, Float.NaN,
            0L,
            !gpsStale,
            latestSpeedEstimateReliable,
            details,
            null
        )
    }

    @Synchronized
    private fun appendRow(
        recordType: String,
        x: Float,
        y: Float,
        z: Float,
        magnitude: Float,
        sensorTimestampNs: Long,
        locationValid: Boolean,
        speedValid: Boolean,
        details: String,
        cameraGeometry: CameraGeometry?,
        includePosition: Boolean = true
    ) {
        if (!::logFile.isInitialized) return

        val wall = System.currentTimeMillis()
        val iso = SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss.SSS", Locale.US).format(Date(wall))

        val lat =
            if (!includePosition || latestLat.isNaN()) ""
            else String.format(Locale.US, "%.7f", latestLat)
        val lon =
            if (!includePosition || latestLon.isNaN()) ""
            else String.format(Locale.US, "%.7f", latestLon)

        val rawSpeed =
            if (latestRawSpeedKmh.isNaN()) "" else String.format(Locale.US, "%.3f", latestRawSpeedKmh)
        val gpsFiltered =
            if (latestGpsFilteredKmh.isNaN()) "" else String.format(Locale.US, "%.3f", latestGpsFilteredKmh)
        val derivedSpeed =
            if (latestDerivedSpeedKmh.isNaN()) "" else String.format(Locale.US, "%.3f", latestDerivedSpeedKmh)
        val fusedSpeed = String.format(Locale.US, "%.3f", latestSpeedKmh)
        val forwardAccel = String.format(Locale.US, "%.4f", latestForwardAccelMps2)
        val bearing =
            if (latestBearingValid) String.format(Locale.US, "%.3f", latestBearing) else ""

        val cg = cameraGeometry
        val camera = cg?.cluster?.camera
        val cameraId =
            if (cg != null) cg.cluster.memberIds.joinToString("|") else activeCameraId
        val cameraType = camera?.type ?: activeCameraType
        val cameraLat = camera?.latitude?.let { String.format(Locale.US, "%.7f", it) } ?: ""
        val cameraLon = camera?.longitude?.let { String.format(Locale.US, "%.7f", it) } ?: ""
        val directDistance = cg?.directDistanceM?.let { String.format(Locale.US, "%.2f", it) } ?: ""
        val vehicleForward = cg?.vehicleForwardM?.let { String.format(Locale.US, "%.2f", it) } ?: ""
        val vehicleCross = cg?.vehicleCrossM?.let { String.format(Locale.US, "%.2f", it) } ?: ""
        val roadCross = cg?.roadCrossM?.let { String.format(Locale.US, "%.2f", it) } ?: ""
        val curveCross = cg?.curveCrossM?.let { String.format(Locale.US, "%.2f", it) } ?: ""
        val bearingTo = cg?.bearingToCameraDeg?.let { String.format(Locale.US, "%.2f", it) } ?: ""
        val forwardDelta = cg?.forwardDeltaDeg?.let { String.format(Locale.US, "%.2f", it) } ?: ""
        val axisDelta = cg?.roadAxisDeltaDeg?.let { String.format(Locale.US, "%.2f", it) } ?: ""
        val directionDelta = cg?.roadDirectionDeltaDeg?.let { String.format(Locale.US, "%.2f", it) } ?: ""
        val hardGeometryOk = cg?.hardGeometryOk?.toString() ?: ""
        val matchScore = cg?.matchScore?.toString() ?: ""
        val requiredScore = cg?.requiredScore?.toString() ?: ""
        val confirmed =
            if (cg != null) {
                (cameraStates[cg.cluster.key]?.phase == TrackPhase.CONFIRMED).toString()
            } else {
                activeCameraId.isNotEmpty().toString()
            }
        val alertLevel =
            if (cg != null) cameraStates[cg.cluster.key]?.alertLevel?.toString() ?: "0"
            else activeCameraAlertLevel.toString()
        val ttc = cg?.ttcS?.takeIf { it >= 0f }?.let { String.format(Locale.US, "%.2f", it) } ?: ""
        val warningDistance = cg?.warningDistanceM?.let { String.format(Locale.US, "%.1f", it) } ?: ""
        val cameraLimit = cg?.candidateLimit?.toString() ?: activeCameraSpeedLimit?.toString() ?: ""

        val sx = if (x.isNaN()) "" else String.format(Locale.US, "%.6f", x)
        val sy = if (y.isNaN()) "" else String.format(Locale.US, "%.6f", y)
        val sz = if (z.isNaN()) "" else String.format(Locale.US, "%.6f", z)
        val sm = if (magnitude.isNaN()) "" else String.format(Locale.US, "%.6f", magnitude)

        val row = listOf(
            wall.toString(),
            iso,
            recordType,
            lat,
            lon,
            latestProvider,
            currentGpsAgeMs().toString(),
            currentGpsSpeedAgeMs().toString(),
            currentGpsCallbackAgeMs().toString(),
            gpsStale.toString(),
            locationValid.toString(),
            speedValid.toString(),
            rawSpeed,
            gpsFiltered,
            derivedSpeed,
            fusedSpeed,
            forwardAccel,
            fusionActive.toString(),
            imuBridgeActive.toString(),
            bearing,
            latestBearingValid.toString(),
            String.format(Locale.US, "%.2f", latestAccuracy),
            gnssSatellites.toString(),
            gnssUsedInFix.toString(),
            cameraId,
            cameraType,
            cameraLat,
            cameraLon,
            directDistance,
            vehicleForward,
            vehicleCross,
            roadCross,
            curveCross,
            bearingTo,
            forwardDelta,
            axisDelta,
            directionDelta,
            hardGeometryOk,
            matchScore,
            requiredScore,
            confirmed,
            alertLevel,
            ttc,
            warningDistance,
            cameraLimit,
            activeLimitKmh.toString(),
            sx,
            sy,
            sz,
            sm,
            sensorTimestampNs.toString(),
            details
        ).joinToString(",") { csvEscape(it) }

        val line = row + "\n"
        try { logFile.appendText(line) } catch (_: Exception) {}

        try {
            liveWriter?.write(line)
            val now = SystemClock.elapsedRealtime()
            if (now - lastLiveFlushElapsed >= LIVE_LOG_FLUSH_MS) {
                liveWriter?.flush()
                lastLiveFlushElapsed = now
            }
        } catch (_: Exception) {}
    }

    private fun csvEscape(s: String): String {
        if (!s.contains(',') && !s.contains('"') && !s.contains('\n')) return s
        return "\"" + s.replace("\"", "\"\"") + "\""
    }

    private fun updatePrefs() {
        prefs.edit()
            .putBoolean("running", serviceTrackingActive)
            .putFloat("speed_kmh", latestSpeedKmh)
            .putBoolean("speed_display_valid", speedDisplayValid)
            .putFloat("raw_speed_kmh", if (latestRawSpeedKmh.isNaN()) -1f else latestRawSpeedKmh)
            .putFloat("gps_filtered_kmh", if (latestGpsFilteredKmh.isNaN()) -1f else latestGpsFilteredKmh)
            .putFloat("derived_speed_kmh", if (latestDerivedSpeedKmh.isNaN()) -1f else latestDerivedSpeedKmh)
            .putFloat("forward_accel_mps2", latestForwardAccelMps2)
            .putBoolean("fusion_active", fusionActive)
            .putBoolean("imu_bridge", imuBridgeActive)
            .putString(
                "lat",
                if (latestLat.isNaN()) null else String.format(Locale.US, "%.6f", latestLat)
            )
            .putString(
                "lon",
                if (latestLon.isNaN()) null else String.format(Locale.US, "%.6f", latestLon)
            )
            .putFloat("accuracy_m", latestAccuracy)
            .putString("provider", latestProvider)
            .putLong("gps_age_ms", currentGpsAgeMs())
            .putLong("gps_speed_age_ms", currentGpsSpeedAgeMs())
            .putLong("gps_callback_age_ms", currentGpsCallbackAgeMs())
            .putBoolean("gps_stale", gpsStale)
            .putInt("gnss_satellites", gnssSatellites)
            .putInt("gnss_used_in_fix", gnssUsedInFix)
            .putInt("location_stall_count", locationStallCount)
            .putInt("active_limit_kmh", activeLimitKmh)
            .putString("camera_id", activeCameraId)
            .putString("camera_type", activeCameraType)
            .putFloat("camera_distance_m", activeCameraDistanceM)
            .putInt("camera_limit_kmh", activeCameraSpeedLimit ?: -1)
            .putString("camera_road_name", activeCameraRoadName)
            .putFloat("camera_curve_cross_m", activeCameraCurveCrossM)
            .putFloat("camera_forward_delta", activeCameraForwardDelta)
            .putInt("camera_alert_level", activeCameraAlertLevel)
            .putFloat("camera_warning_distance_m", activeCameraWarningDistanceM)
            .putFloat("camera_ttc_s", activeCameraTtcS)
            .putInt("confirmed_camera_count", confirmedCameraCount)
            .putInt("camera_cluster_count", cameraClusters.size)
            .apply()
    }

    private fun updateNotification() {
        val cameraPart =
            if (activeCameraId.isNotEmpty() && activeCameraDistanceM >= 0f) {
                " • camera ${activeCameraDistanceM.roundToInt()}m L$activeCameraAlertLevel"
            } else {
                ""
            }

        val gpsPart = when {
            gpsStale -> " • GPS stale"
            imuBridgeActive -> " • speed bridge"
            fusionActive -> " • GPS+IMU"
            else -> ""
        }

        val speedPart =
            if (speedDisplayValid) String.format(Locale.US, "%.0f km/h", latestSpeedKmh)
            else "-- km/h"

        val text = "$speedPart • limit $activeLimitKmh$cameraPart$gpsPart"

        val nm = getSystemService(Context.NOTIFICATION_SERVICE) as NotificationManager
        nm.notify(NOTIFICATION_ID, makeNotification(text))
    }

    private fun makeNotification(text: String): Notification {
        val openApp = Intent(this, MainActivity::class.java)
        val pending = PendingIntent.getActivity(
            this,
            0,
            openApp,
            PendingIntent.FLAG_UPDATE_CURRENT or PendingIntent.FLAG_IMMUTABLE
        )

        return Notification.Builder(this, CHANNEL_ID)
            .setContentTitle("MOJO Drive active • persistent logging")
            .setContentText(text)
            .setSmallIcon(android.R.drawable.ic_menu_mylocation)
            .setOngoing(true)
            .setContentIntent(pending)
            .build()
    }

    private fun createNotificationChannel() {
        if (Build.VERSION.SDK_INT >= 26) {
            val channel = NotificationChannel(
                CHANNEL_ID,
                "Driving tracking",
                NotificationManager.IMPORTANCE_LOW
            ).apply {
                description = "Keeps MOJO Drive active while the phone is locked."
            }
            (getSystemService(Context.NOTIFICATION_SERVICE) as NotificationManager)
                .createNotificationChannel(channel)
        }
    }

    override fun onProviderEnabled(provider: String) {
        if (provider == LocationManager.GPS_PROVIDER && serviceTrackingActive) {
            appendMarker("GPS_PROVIDER_ENABLED")
            startLegacyLocationFallback("provider_enabled")
        }
    }

    override fun onProviderDisabled(provider: String) {
        if (provider == LocationManager.GPS_PROVIDER) {
            appendMarker("GPS_PROVIDER_DISABLED")
        }
    }

    override fun onTaskRemoved(rootIntent: Intent?) {
        if (prefs.getBoolean("trip_active", false) && !userStopRequested) {
            appendMarker("TASK_REMOVED_RESTART_REQUESTED")
            try {
                val restart = Intent(applicationContext, LocationService::class.java)
                    .putExtra(EXTRA_THRESHOLD_KMH, prefs.getInt("threshold_kmh", manualThresholdKmh))
                if (Build.VERSION.SDK_INT >= 26) {
                    applicationContext.startForegroundService(restart)
                } else {
                    applicationContext.startService(restart)
                }
            } catch (e: Exception) {
                appendMarker("TASK_REMOVED_RESTART_FAILED", "error=${e.javaClass.simpleName}")
            }
        }
        super.onTaskRemoved(rootIntent)
    }

    override fun onDestroy() {
        serviceTrackingActive = false
        handler.removeCallbacks(healthTicker)

        try {
            if (fusedUpdatesRegistered) {
                fusedLocationClient.removeLocationUpdates(fusedLocationCallback)
                fusedUpdatesRegistered = false
            }
        } catch (_: Exception) {}

        try {
            locationManager.removeUpdates(this)
            legacyLocationUpdatesRegistered = false
            networkFallbackRegistered = false
        } catch (_: Exception) {}

        if (gnssRegistered) {
            try { locationManager.unregisterGnssStatusCallback(gnssCallback) } catch (_: Exception) {}
            gnssRegistered = false
        }

        try {
            sensorManager.unregisterListener(this)
            sensorsRegistered = false
        } catch (_: Exception) {}

        if (loggingStarted && !userStopRequested) {
            appendMarker("SERVICE_INTERRUPTED", "trip_will_be_recovered=true")
            try { liveWriter?.flush() } catch (_: Exception) {}
            closeLiveWriter()

            prefs.edit()
                .putBoolean("running", false)
                .putBoolean("trip_active", true)
                .putString("active_log_name", logName)
                .putString("active_log_uri", liveLogUri?.toString())
                .apply()
        }

        try {
            if (wakeLock?.isHeld == true) wakeLock?.release()
        } catch (_: Exception) {}

        try { toneLow?.release() } catch (_: Exception) {}
        try { toneMid?.release() } catch (_: Exception) {}
        try { toneHigh?.release() } catch (_: Exception) {}

        prefs.edit().putBoolean("running", false).apply()
        super.onDestroy()
    }

    override fun onBind(intent: Intent?): IBinder? = null
}
