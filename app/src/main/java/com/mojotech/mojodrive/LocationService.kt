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
import android.location.Location
import android.location.LocationListener
import android.location.LocationManager
import android.media.AudioManager
import android.media.ToneGenerator
import android.net.Uri
import android.os.*
import android.provider.MediaStore
import android.widget.Toast
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

        private const val GPS_STALE_MS = 8000L
        private const val GPS_CAMERA_BRIDGE_START_MS = 1500L
        private const val GPS_CAMERA_BRIDGE_MAX_MS = 7000L
        private const val FUSION_BRIDGE_MS = 8000L

        private const val ROAD_IMPACT_COOLDOWN_MS = 1200L
        private const val ROAD_IMPACT_SHOCK_MPS2 = 3.2f

        private const val OVERSPEED_REPEAT_MS = 7000L

        private const val CAMERA_SEARCH_M = 2200f
        private const val CAMERA_CONFIRM_EXTRA_M = 450f
        private const val CAMERA_CLUSTER_DISTANCE_M = 40f
        private const val CAMERA_CLUSTER_AXIS_DELTA_DEG = 30f
        private const val CAMERA_CLUSTER_DIRECTED_DELTA_DEG = 38f

        private const val CAMERA_SOFT_CROSS_BASE_M = 48f
        private const val CAMERA_SOFT_CROSS_MAX_M = 72f
        private const val CAMERA_DIAG_NEAR_M = 900f
        private const val CAMERA_DIAG_INTERVAL_MS = 5000L

        private const val PASS_BEHIND_DEG = 100f
        private const val PASS_DISTANCE_GROWTH_M = 25f
        private const val NEAR_PASS_AUDIT_M = 140f

        private const val GLOBAL_ALERT_SWITCH_MIN_MS = 650L

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
        val roadAxisDeltaDeg: Float,
        val softCrossLimitM: Float,
        val hardCrossLimitM: Float,
        val hardCrossOk: Boolean,
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
        var bridgeLogged: Boolean = false
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
    private val rotationMatrix = FloatArray(9)

    private var rotationReady = false
    private var fusionInitialized = false
    private var fusionSpeedMps = 0f
    private var fusionVariance = 4f
    private var lastFusionSensorNs = 0L
    private var latestGpsFilteredKmh = Float.NaN
    private var latestForwardAccelMps2 = 0f
    private var fusionActive = false
    private var imuBridgeActive = false

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

    @Volatile private var latestLat = Double.NaN
    @Volatile private var latestLon = Double.NaN
    @Volatile private var latestRawSpeedKmh = Float.NaN
    @Volatile private var latestSpeedKmh = 0f
    @Volatile private var latestBearing = -1f
    @Volatile private var latestBearingValid = false
    @Volatile private var latestAccuracy = -1f
    @Volatile private var latestProvider = "--"
    @Volatile private var lastValidGpsElapsedMs = 0L
    @Volatile private var gpsStale = true

    @Volatile private var activeCameraId = ""
    @Volatile private var activeCameraType = ""
    @Volatile private var activeCameraDistanceM = -1f
    @Volatile private var activeCameraSpeedLimit: Int? = null
    @Volatile private var activeCameraRoadName = ""
    @Volatile private var activeCameraRoadCrossM = -1f
    @Volatile private var activeCameraForwardDelta = -1f
    @Volatile private var activeCameraAlertLevel = 0
    @Volatile private var activeCameraTtcS = -1f
    @Volatile private var activeCameraWarningDistanceM = -1f
    @Volatile private var confirmedCameraCount = 0

    private val staleTicker = object : Runnable {
        override fun run() {
            updateGpsStaleState()
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
        handler.post(staleTicker)
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

        if (!loggingStarted && !resumeInterruptedTrip()) {
            startNewTripLog()
        }

        startForeground(NOTIFICATION_ID, makeNotification("Waiting for valid GPS…"))
        prefs.edit().putBoolean("running", true).apply()

        startLocationUpdates()
        startSensorUpdates()

        if (loggingStarted) {
            appendMarker(
                "CAMERA_DB_READY",
                "raw=${cameras.size};clusters=${cameraClusters.size};" +
                    "db=shiraz_cameras_v09;engine=clustered_confidence_v09"
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
        Location.distanceBetween(
            a.latitude, a.longitude,
            b.latitude, b.longitude,
            results
        )

        if (results[0] > CAMERA_CLUSTER_DISTANCE_M) return false

        val aRb = a.roadBearing
        val bRb = b.roadBearing

        if (aRb != null && bRb != null) {
            val aDirected = hasReliableDirectedBearing(a)
            val bDirected = hasReliableDirectedBearing(b)

            if (aDirected && bDirected) {
                if (angleDifference(aRb, bRb) > CAMERA_CLUSTER_DIRECTED_DELTA_DEG) {
                    return false
                }
            } else {
                val axisDelta = min(
                    angleDifference(aRb, bRb),
                    angleDifference(aRb, normalizeBearing(bRb + 180f))
                )
                if (axisDelta > CAMERA_CLUSTER_AXIS_DELTA_DEG) return false
            }
        }

        return true
    }

    private fun hasReliableDirectedBearing(camera: CameraPoint): Boolean {
        if (!camera.oneway || camera.roadBearing == null) return false
        val c = camera.directionConfidence.lowercase(Locale.US)
        return c == "high" || c == "medium"
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
        "wall_time_ms,iso_time,record_type,lat,lon,provider,gps_age_ms,gps_stale,speed_valid," +
            "raw_speed_kmh,gps_filtered_kmh,fused_speed_kmh,forward_accel_mps2,fusion_active,imu_bridge," +
            "bearing_deg,bearing_valid,gps_accuracy_m,camera_id,camera_type,camera_lat,camera_lon," +
            "camera_direct_distance_m,camera_forward_m,camera_vehicle_cross_m,camera_road_cross_m," +
            "camera_bearing_to_deg,camera_forward_delta_deg,camera_axis_delta_deg,camera_match_score," +
            "camera_required_score,camera_confirmed,camera_alert_level,camera_ttc_s," +
            "camera_warning_distance_m,camera_limit_kmh,active_limit_kmh,x,y,z,magnitude," +
            "sensor_timestamp_ns,details\n"

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
            "persistent_live_log=true;camera_engine=clustered_confidence_v09;raw_sensor_csv=false"
        )
    }

    private fun resumeInterruptedTrip(): Boolean {
        if (!prefs.getBoolean("trip_active", false)) return false

        val name = prefs.getString("active_log_name", null) ?: return false
        val internal = File(filesDir, name)
        if (!internal.exists() || internal.length() == 0L) return false

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
                put(
                    MediaStore.Downloads.RELATIVE_PATH,
                    Environment.DIRECTORY_DOWNLOADS + "/MOJODrive"
                )
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
            liveWriter = BufferedWriter(
                OutputStreamWriter(contentResolver.openOutputStream(uri, "wa")!!)
            )
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


    private fun startLocationUpdates() {
        if (
            checkSelfPermission(Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED &&
            checkSelfPermission(Manifest.permission.ACCESS_COARSE_LOCATION) != PackageManager.PERMISSION_GRANTED
        ) {
            stopSelf()
            return
        }

        try {
            if (locationManager.isProviderEnabled(LocationManager.GPS_PROVIDER)) {
                locationManager.requestLocationUpdates(
                    LocationManager.GPS_PROVIDER,
                    500L,
                    0f,
                    this
                )
            }
        } catch (_: Exception) {}
    }

    private fun startSensorUpdates() {
        accelerometer?.let { sensorManager.registerListener(this, it, 20_000) }
        gyroscope?.let { sensorManager.registerListener(this, it, 20_000) }
        linearAcceleration?.let { sensorManager.registerListener(this, it, 20_000) }
        rotationVector?.let {
            sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME)
        }
    }

    override fun onLocationChanged(location: Location) {
        if (location.provider != LocationManager.GPS_PROVIDER) return

        latestLat = location.latitude
        latestLon = location.longitude
        latestProvider = location.provider ?: "gps"
        latestAccuracy = if (location.hasAccuracy()) location.accuracy else -1f

        val ageMs = locationAgeMs(location)
        val rawSpeed = if (location.hasSpeed()) location.speed * 3.6f else Float.NaN
        latestRawSpeedKmh = rawSpeed

        val speedAccuracyOk =
            if (Build.VERSION.SDK_INT >= 26 && location.hasSpeedAccuracy()) {
                location.speedAccuracyMetersPerSecond <= 5.5f
            } else {
                true
            }

        val accepted =
            location.hasSpeed() &&
                ageMs in 0..3000 &&
                (!location.hasAccuracy() || location.accuracy <= 50f) &&
                speedAccuracyOk &&
                rawSpeed in 0f..250f

        if (accepted) {
            latestGpsFilteredKmh = filteredSpeed(rawSpeed)
            lastValidGpsElapsedMs = SystemClock.elapsedRealtime()
            gpsStale = false
            imuBridgeActive = false

            latestBearingValid = location.hasBearing() && latestGpsFilteredKmh >= 8f
            if (latestBearingValid) latestBearing = location.bearing

            val gpsSpeedAccuracyMps =
                if (Build.VERSION.SDK_INT >= 26 && location.hasSpeedAccuracy()) {
                    location.speedAccuracyMetersPerSecond.coerceIn(0.3f, 5.5f)
                } else {
                    2.0f
                }

            correctFusionWithGps(
                latestGpsFilteredKmh / 3.6f,
                gpsSpeedAccuracyMps
            )

            latestSpeedKmh = (fusionSpeedMps * 3.6f).coerceIn(0f, 250f)

            evaluateAllCameras()
            handleOverspeed(latestSpeedKmh)

            appendRow(
                "GPS",
                Float.NaN, Float.NaN, Float.NaN, Float.NaN,
                0L,
                true,
                "fusion=${if (fusionActive) "GPS_IMU" else "GPS"}",
                null
            )
        } else {
            appendRow(
                "GPS_REJECTED",
                Float.NaN, Float.NaN, Float.NaN, Float.NaN,
                0L,
                false,
                rejectionReason(location, ageMs, speedAccuracyOk),
                null
            )
        }

        updatePrefs()
        updateNotification()
    }

    private fun rejectionReason(
        location: Location,
        ageMs: Long,
        speedAccuracyOk: Boolean
    ): String {
        return when {
            !location.hasSpeed() -> "NO_SPEED"
            ageMs !in 0..3000 -> "STALE_FIX"
            location.hasAccuracy() && location.accuracy > 50f -> "POOR_ACCURACY"
            !speedAccuracyOk -> "POOR_SPEED_ACCURACY"
            else -> "INVALID_SPEED"
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

    private fun correctFusionWithGps(
        gpsSpeedMps: Float,
        speedAccuracyMps: Float
    ) {
        if (!fusionInitialized) {
            fusionSpeedMps = gpsSpeedMps.coerceAtLeast(0f)
            fusionVariance = max(1f, speedAccuracyMps * speedAccuracyMps)
            fusionInitialized = true
            fusionActive = linearAcceleration != null && rotationVector != null
            lastFusionSensorNs = 0L
            return
        }

        val measurementVariance = max(0.25f, speedAccuracyMps * speedAccuracyMps)
        val gain = fusionVariance / (fusionVariance + measurementVariance)

        fusionSpeedMps += gain * (gpsSpeedMps - fusionSpeedMps)
        fusionSpeedMps = fusionSpeedMps.coerceIn(0f, 70f)
        fusionVariance = max(0.05f, (1f - gain) * fusionVariance)
        fusionActive = linearAcceleration != null && rotationVector != null
    }

    private fun predictFusion(
        forwardAccelMps2: Float,
        sensorTimestampNs: Long
    ) {
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

        val age = currentGpsAgeMs()
        if (age < 0L || age > FUSION_BRIDGE_MS) return

        val a =
            if (abs(forwardAccelMps2) < 0.08f) {
                0f
            } else {
                forwardAccelMps2.coerceIn(-8f, 8f)
            }

        fusionSpeedMps = (fusionSpeedMps + a * dt).coerceIn(0f, 70f)
        fusionVariance = (fusionVariance + (0.7f * 0.7f * dt)).coerceAtMost(25f)

        latestForwardAccelMps2 = a
        latestSpeedKmh = (fusionSpeedMps * 3.6f).coerceIn(0f, 250f)
        imuBridgeActive = age > 1200L && latestSpeedKmh >= 5f
        fusionActive = true

        if (!gpsStale) handleOverspeed(latestSpeedKmh)
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

    private fun updateGpsStaleState() {
        val age = currentGpsAgeMs()
        val staleNow = age < 0L || age > GPS_STALE_MS

        if (staleNow != gpsStale) {
            gpsStale = staleNow
            if (gpsStale && loggingStarted) appendMarker("GPS_STALE")
        }

        imuBridgeActive =
            fusionActive &&
                !gpsStale &&
                age > 1200L &&
                latestSpeedKmh >= 5f

        if (age in GPS_CAMERA_BRIDGE_START_MS..GPS_CAMERA_BRIDGE_MAX_MS) {
            bridgeConfirmedCameraAlerts(age)
        }

        updatePrefs()
    }

    private fun currentGpsAgeMs(): Long {
        if (lastValidGpsElapsedMs <= 0L) return -1L
        return (SystemClock.elapsedRealtime() - lastValidGpsElapsedMs).coerceAtLeast(0L)
    }

    private fun evaluateAllCameras() {
        cameraAlertZoneActive = false

        if (
            latestLat.isNaN() ||
            latestLon.isNaN() ||
            cameraClusters.isEmpty() ||
            !latestBearingValid
        ) {
            clearUiCamera()
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

            if (
                state.phase == TrackPhase.PASSED ||
                state.phase == TrackPhase.ABANDONED
            ) {
                state.lastSeenElapsed = now
                continue
            }

            val geometry = buildCameraGeometry(from, to, cluster, state, now)

            state.lastSeenElapsed = now
            state.minDistanceM = min(state.minDistanceM, geometry.directDistanceM)

            updateApproachState(state, geometry)

            val withinConfirmRange =
                geometry.directDistanceM <=
                    max(
                        900f,
                        geometry.warningDistanceM + CAMERA_CONFIRM_EXTRA_M
                    )

            val strongApproachHistory = state.approachHits >= 2

            val nearFastTrack =
                geometry.directDistanceM <= 450f &&
                    strongApproachHistory &&
                    geometry.hardCrossOk &&
                    geometry.forwardDeltaDeg <= 78f &&
                    geometry.matchScore >= geometry.requiredScore - 1

            val confirmEligible =
                withinConfirmRange &&
                    geometry.hardCrossOk &&
                    geometry.vehicleForwardM > 8f &&
                    (
                        geometry.matchScore >= geometry.requiredScore ||
                        nearFastTrack
                    )

            if (state.phase == TrackPhase.TRACKING) {
                if (confirmEligible) {
                    state.confirmHits++

                    val hitsNeeded =
                        if (nearFastTrack) 1 else geometry.requiredHits

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

                if (geometry.hardCrossOk) {
                    state.hardBadHits = 0
                } else {
                    state.hardBadHits++
                }

                val clearlyBehind =
                    geometry.forwardDeltaDeg >= PASS_BEHIND_DEG

                val distanceGrowing =
                    state.lastDistanceM != Float.MAX_VALUE &&
                        geometry.directDistanceM >
                            state.minDistanceM + PASS_DISTANCE_GROWTH_M

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
                        "reason=hard_cross_failed_while_moving_away;hardBadHits=${state.hardBadHits}"
                    )

                    state.lastDistanceM = geometry.directDistanceM
                    continue
                }

                confirmedGeometries += geometry

                computeAlertRequest(
                    geometry,
                    state,
                    false
                )?.let {
                    alertRequests += it
                }
            }

            state.lastDistanceM = geometry.directDistanceM
        }

        confirmedCameraCount =
            confirmedGeometries.count { it.vehicleForwardM > 0f }

        chooseUiCamera(confirmedGeometries)

        cameraAlertZoneActive = alertRequests.isNotEmpty()
        dispatchBestCameraAlert(alertRequests)

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

        val bearingToCamera =
            normalizeBearing(from.bearingTo(to))

        val forwardDelta =
            angleDifference(latestBearing, bearingToCamera)

        val vehicleForward =
            (
                distance *
                    cos(Math.toRadians(forwardDelta.toDouble()))
                ).toFloat()

        val vehicleCross =
            abs(
                distance *
                    sin(Math.toRadians(forwardDelta.toDouble()))
            ).toFloat()

        val rb = camera.roadBearing

        val roadCross =
            if (rb != null) {
                val axisToCamera =
                    min(
                        angleDifference(normalizeBearing(rb), bearingToCamera),
                        angleDifference(normalizeBearing(rb + 180f), bearingToCamera)
                    )

                abs(
                    distance *
                        sin(Math.toRadians(axisToCamera.toDouble()))
                ).toFloat()
            } else {
                vehicleCross
            }

        val roadAxisDelta =
            if (rb != null) {
                min(
                    angleDifference(latestBearing, normalizeBearing(rb)),
                    angleDifference(latestBearing, normalizeBearing(rb + 180f))
                )
            } else {
                0f
            }

        val softCrossLimit =
            (
                CAMERA_SOFT_CROSS_BASE_M +
                    max(0f, latestAccuracy) * 1.4f
                ).coerceIn(
                    CAMERA_SOFT_CROSS_BASE_M,
                    CAMERA_SOFT_CROSS_MAX_M
                )

        val curveAllowance =
            when {
                distance <= 250f -> 0f
                distance <= 600f -> (distance - 250f) * 0.10f
                distance <= 1000f -> 35f + (distance - 600f) * 0.10f
                else -> 75f + (distance - 1000f) * 0.05f
            }

        val hardCrossLimit =
            (softCrossLimit + curveAllowance)
                .coerceIn(65f, 190f)

        val hardCrossOk =
            roadCross <= hardCrossLimit

        val candidateLimit =
            if (camera.type == "speed") validDynamicLimit(camera) else null

        val warningDistance =
            computeAdaptiveWarningDistanceM(
                latestSpeedKmh,
                candidateLimit,
                camera.type
            )

        val speedMps = max(1.5f, latestSpeedKmh / 3.6f)

        val ttc =
            if (vehicleForward > 0f) vehicleForward / speedMps else -1f

        val dtS =
            if (state.lastSeenElapsed > 0L) {
                (now - state.lastSeenElapsed).coerceAtLeast(1L) / 1000f
            } else {
                0f
            }

        val approachDelta =
            if (state.lastDistanceM != Float.MAX_VALUE) {
                state.lastDistanceM - distance
            } else {
                0f
            }

        val approachRate =
            if (dtS > 0f) approachDelta / dtS else 0f

        var score = 0
        val reasons = ArrayList<String>()

        if (!hardCrossOk) {
            reasons += "hard_cross"
        } else {
            when {
                roadCross <= softCrossLimit * 0.65f -> score += 3
                roadCross <= softCrossLimit -> score += 2
                else -> score += 1
            }
        }

        if (rb != null) {
            when {
                roadAxisDelta <= 22f -> score += 2
                roadAxisDelta <= 48f -> score += 1
                else -> reasons += "road_axis"
            }
        } else {
            score += 1
        }

        when {
            forwardDelta <= 35f -> score += 3
            forwardDelta <= 60f -> score += 2
            forwardDelta <= 85f -> score += 1
            else ->
                reasons +=
                    if (forwardDelta >= PASS_BEHIND_DEG) {
                        "behind_vehicle"
                    } else {
                        "forward_angle"
                    }
        }

        if (vehicleForward > 10f) {
            score += 1
        } else {
            reasons += "not_ahead"
        }

        if (approachDelta > 2f) {
            score += 2
        } else if (approachDelta < -3f) {
            reasons += "moving_away"
        }

        when {
            latestAccuracy in 0f..8f -> score += 2
            latestAccuracy in 0f..18f -> score += 1
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
                distance > 450f -> 2
                else -> 2
            }

        return CameraGeometry(
            cluster = cluster,
            directDistanceM = distance,
            vehicleForwardM = vehicleForward,
            vehicleCrossM = vehicleCross,
            bearingToCameraDeg = bearingToCamera,
            forwardDeltaDeg = forwardDelta,
            roadCrossM = roadCross,
            roadAxisDeltaDeg = roadAxisDelta,
            softCrossLimitM = softCrossLimit,
            hardCrossLimitM = hardCrossLimit,
            hardCrossOk = hardCrossOk,
            warningDistanceM = warningDistance,
            ttcS = ttc,
            candidateLimit = candidateLimit,
            approachDeltaM = approachDelta,
            approachRateMps = approachRate,
            matchScore = score,
            requiredScore = requiredScore,
            requiredHits = requiredHits,
            matchReason =
                if (reasons.isEmpty()) "good"
                else reasons.joinToString("+")
        )
    }

    private fun updateApproachState(
        state: CameraTrackState,
        geometry: CameraGeometry
    ) {
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

        if (
            geometry.vehicleForwardM <= 0f ||
            geometry.forwardDeltaDeg >= PASS_BEHIND_DEG
        ) {
            state.alertLevel = 0
            return null
        }

        // Hard cross is never bypassed by score.
        if (!geometry.hardCrossOk) {
            state.alertLevel = 0
            return null
        }

        val inWarningZone =
            geometry.vehicleForwardM <=
                geometry.warningDistanceM + 90f

        if (!inWarningZone) {
            state.alertLevel = 0
            return null
        }

        val level =
            computeCameraAlertLevel(
                geometry.vehicleForwardM,
                geometry.ttcS,
                latestSpeedKmh,
                geometry.candidateLimit,
                geometry.warningDistanceM
            )

        state.alertLevel = level

        if (!state.warned) {
            state.warned = true
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

        return AlertRequest(
            geometry = geometry,
            state = state,
            level = level,
            bridge = bridge
        )
    }

    private fun dispatchBestCameraAlert(
        requests: List<AlertRequest>
    ) {
        if (requests.isEmpty()) return

        val best =
            requests.sortedWith(
                compareByDescending<AlertRequest> { it.level }
                    .thenBy {
                        if (it.geometry.ttcS > 0f) {
                            it.geometry.ttcS
                        } else {
                            Float.MAX_VALUE
                        }
                    }
                    .thenBy { it.geometry.vehicleForwardM }
            ).first()

        val now = SystemClock.elapsedRealtime()

        val interval =
            when (best.level) {
                1 -> 3000L
                2 -> 1800L
                3 -> 900L
                4 -> 450L
                else -> Long.MAX_VALUE
            }

        val switchedCluster =
            best.geometry.cluster.key != globalLastCameraKey

        val switchReady =
            switchedCluster &&
                now - globalLastCameraPulseElapsed >= GLOBAL_ALERT_SWITCH_MIN_MS

        val cadenceReady =
            now - globalLastCameraPulseElapsed >= interval

        if (!switchReady && !cadenceReady) return

        globalLastCameraPulseElapsed = now
        globalLastCameraKey = best.geometry.cluster.key

        dispatchCameraAlertOutput(
            best,
            max(0, requests.size - 1)
        )
    }

    private fun bridgeConfirmedCameraAlerts(
        gpsAgeMs: Long
    ) {
        if (
            gpsAgeMs < GPS_CAMERA_BRIDGE_START_MS ||
            gpsAgeMs > GPS_CAMERA_BRIDGE_MAX_MS
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
            if (!last.hardCrossOk) continue

            val elapsedS =
                (now - state.lastGeometryElapsed)
                    .coerceAtLeast(0L) / 1000f

            if (
                elapsedS <= 0f ||
                elapsedS >
                    GPS_CAMERA_BRIDGE_MAX_MS / 1000f
            ) {
                continue
            }

            val predictedForward =
                last.vehicleForwardM -
                    speedMps * elapsedS

            if (predictedForward <= 0f) continue

            val predictedDirect =
                max(
                    0f,
                    last.directDistanceM -
                        speedMps * elapsedS
                )

            val predictedTtc =
                predictedForward /
                    max(1.5f, speedMps)

            val bridged =
                last.copy(
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
                    "gpsAgeMs=$gpsAgeMs"
                )
            }

            computeAlertRequest(
                bridged,
                state,
                true
            )?.let {
                requests += it
            }
        }

        if (requests.isNotEmpty()) {
            cameraAlertZoneActive = true
            dispatchBestCameraAlert(requests)
        }
    }

    private fun chooseUiCamera(
        geometries: List<CameraGeometry>
    ) {
        val ahead =
            geometries.filter {
                it.vehicleForwardM > 0f &&
                    it.forwardDeltaDeg < PASS_BEHIND_DEG &&
                    it.hardCrossOk
            }

        if (ahead.isEmpty()) {
            clearUiCamera()
            return
        }

        val best =
            ahead.minByOrNull {
                if (it.ttcS > 0f) it.ttcS
                else Float.MAX_VALUE
            } ?: run {
                clearUiCamera()
                return
            }

        val state =
            cameraStates[best.cluster.key]

        activeCameraId =
            if (best.cluster.memberIds.size > 1) {
                best.cluster.camera.id +
                    "(+${best.cluster.memberIds.size - 1})"
            } else {
                best.cluster.camera.id
            }

        activeCameraType = best.cluster.camera.type
        activeCameraDistanceM = best.vehicleForwardM
        activeCameraSpeedLimit = best.candidateLimit
        activeCameraRoadName = best.cluster.camera.roadName
        activeCameraRoadCrossM = best.roadCrossM
        activeCameraForwardDelta = best.forwardDeltaDeg
        activeCameraAlertLevel = state?.alertLevel ?: 0
        activeCameraTtcS = best.ttcS
        activeCameraWarningDistanceM = best.warningDistanceM

        activeLimitKmh =
            if (
                best.vehicleForwardM <=
                    best.warningDistanceM + 90f &&
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
        activeCameraRoadCrossM = -1f
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
            if (
                now - entry.value.lastSeenElapsed >
                180_000L
            ) {
                iterator.remove()
            }
        }
    }

    private fun computeAdaptiveWarningDistanceM(
        speedKmh: Float,
        cameraLimitKmh: Int?,
        cameraType: String
    ): Float {

        val v = max(5f, speedKmh) / 3.6f

        val baseLeadS =
            (
                24f +
                    (speedKmh - 45f)
                        .coerceAtLeast(0f) *
                        0.12f
                ).coerceIn(24f, 32f)

        var warningM = v * baseLeadS

        if (
            cameraLimitKmh != null &&
            speedKmh > cameraLimitKmh
        ) {
            val target =
                cameraLimitKmh / 3.6f

            val decelDistance =
                (
                    (v * v - target * target) /
                        (2f * 1.6f)
                    ).coerceAtLeast(0f)

            val reactionDistance = v * 5f

            warningM =
                max(
                    warningM,
                    decelDistance +
                        reactionDistance +
                        90f
                )
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

        val overLimit =
            cameraLimitKmh != null &&
                speedKmh >
                    cameraLimitKmh + 2f

        return when {
            (
                ttcS in 0f..6f ||
                    forwardM <= 140f
                ) &&
                overLimit ->
                4

            ttcS in 0f..9f ||
                forwardM <= 220f ->
                3

            ttcS in 0f..15f ||
                forwardM <=
                    min(
                        420f,
                        warningDistanceM * 0.72f
                    ) ->
                2

            else ->
                1
        }
    }

    private fun dispatchCameraAlertOutput(
        request: AlertRequest,
        suppressedCount: Int
    ) {
        val geometry = request.geometry
        val level = request.level

        val tone =
            when (level) {
                1 -> toneLow
                2 -> toneMid
                else -> toneHigh
            }

        val duration =
            when (level) {
                1 -> 120
                2 -> 170
                3 -> 240
                else -> 360
            }

        val toneId =
            if (
                geometry.cluster.camera.type ==
                "red_light"
            ) {
                ToneGenerator.TONE_PROP_ACK
            } else {
                ToneGenerator.TONE_PROP_BEEP2
            }

        val toneOk =
            try {
                tone?.startTone(
                    toneId,
                    duration
                ) ?: false
            } catch (_: Exception) {
                false
            }

        val vibrator =
            getSystemService(Context.VIBRATOR_SERVICE) as Vibrator

        val vibratorPresent =
            try {
                vibrator.hasVibrator()
            } catch (_: Exception) {
                false
            }

        var vibrationDispatched = false

        if (vibratorPresent) {
            try {
                when (level) {
                    1 ->
                        vibratePattern(
                            longArrayOf(0, 70),
                            intArrayOf(0, 120)
                        )

                    2 ->
                        vibratePattern(
                            longArrayOf(0, 110),
                            intArrayOf(0, 180)
                        )

                    3 ->
                        vibratePattern(
                            longArrayOf(0, 140),
                            intArrayOf(0, 230)
                        )

                    else ->
                        vibratePattern(
                            longArrayOf(0, 200, 70, 200),
                            intArrayOf(0, 255, 0, 255)
                        )
                }

                vibrationDispatched = true
            } catch (_: Exception) {}
        }

        val alarmVolume =
            try {
                audioManager.getStreamVolume(
                    AudioManager.STREAM_ALARM
                )
            } catch (_: Exception) {
                -1
            }

        val alarmMax =
            try {
                audioManager.getStreamMaxVolume(
                    AudioManager.STREAM_ALARM
                )
            } catch (_: Exception) {
                -1
            }

        appendCameraEvent(
            "ALERT_OUTPUT",
            geometry,
            request.state,
            "level=$level;" +
                "bridge=${request.bridge};" +
                "tone_start_ok=$toneOk;" +
                "vibrator_present=$vibratorPresent;" +
                "vibration_dispatched=$vibrationDispatched;" +
                "alarm_volume=$alarmVolume;" +
                "alarm_max=$alarmMax;" +
                "suppressed_other_camera_alerts=$suppressedCount;" +
                "stream=ALARM;" +
                "duration_ms=$duration"
        )
    }

    private fun validDynamicLimit(
        camera: CameraPoint
    ): Int? {

        val limit = camera.speedLimit ?: return null
        val confidence =
            camera.speedConfidence.lowercase(Locale.US)

        if (limit !in 20..150) return null
        if (
            confidence != "high" &&
            confidence != "medium"
        ) return null
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
            "cluster=${geometry.cluster.key};" +
                "memberIds=${geometry.cluster.memberIds.joinToString("|")};" +
                "direct=${String.format(Locale.US, "%.1f", geometry.directDistanceM)};" +
                "forward=${String.format(Locale.US, "%.1f", geometry.vehicleForwardM)};" +
                "vehicleCross=${String.format(Locale.US, "%.1f", geometry.vehicleCrossM)};" +
                "roadCross=${String.format(Locale.US, "%.1f", geometry.roadCrossM)};" +
                "softCross=${String.format(Locale.US, "%.1f", geometry.softCrossLimitM)};" +
                "hardCross=${String.format(Locale.US, "%.1f", geometry.hardCrossLimitM)};" +
                "hardCrossOk=${geometry.hardCrossOk};" +
                "bearingTo=${String.format(Locale.US, "%.1f", geometry.bearingToCameraDeg)};" +
                "forwardDelta=${String.format(Locale.US, "%.1f", geometry.forwardDeltaDeg)};" +
                "axisDelta=${String.format(Locale.US, "%.1f", geometry.roadAxisDeltaDeg)};" +
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
        if (gpsStale) return

        if (
            abs(activeLimitKmh - lastAlertLimitKmh) >= 3
        ) {
            overspeedActive = false
            lastAlertLimitKmh = activeLimitKmh
        }

        val now = SystemClock.elapsedRealtime()

        if (speedKmh >= activeLimitKmh + 1f) {
            if (
                !overspeedActive ||
                now - lastOverspeedAlertElapsed >= OVERSPEED_REPEAT_MS
            ) {
                overspeedActive = true
                lastOverspeedAlertElapsed = now

                if (!cameraAlertZoneActive) {
                    overspeedAlert()
                }

                appendMarker(
                    "OVERSPEED_ALERT",
                    "speed=${speedKmh.roundToInt()};limit=$activeLimitKmh;camera=$activeCameraId"
                )
            }
        } else if (
            speedKmh <= activeLimitKmh - 4f
        ) {
            overspeedActive = false
        }
    }

    private fun overspeedAlert() {
        val toneOk =
            try {
                toneHigh?.startTone(
                    ToneGenerator.TONE_PROP_BEEP2,
                    500
                ) ?: false
            } catch (_: Exception) {
                false
            }

        val vibrator =
            getSystemService(Context.VIBRATOR_SERVICE) as Vibrator

        val hasVibrator =
            try {
                vibrator.hasVibrator()
            } catch (_: Exception) {
                false
            }

        var vibrationDispatched = false

        if (hasVibrator) {
            try {
                vibratePattern(
                    longArrayOf(0, 300, 100, 300),
                    intArrayOf(0, 255, 0, 255)
                )
                vibrationDispatched = true
            } catch (_: Exception) {}
        }

        val alarmVolume =
            try {
                audioManager.getStreamVolume(
                    AudioManager.STREAM_ALARM
                )
            } catch (_: Exception) {
                -1
            }

        val alarmMax =
            try {
                audioManager.getStreamMaxVolume(
                    AudioManager.STREAM_ALARM
                )
            } catch (_: Exception) {
                -1
            }

        appendMarker(
            "OVERSPEED_OUTPUT",
            "tone_start_ok=$toneOk;vibrator_present=$hasVibrator;" +
                "vibration_dispatched=$vibrationDispatched;" +
                "alarm_volume=$alarmVolume;alarm_max=$alarmMax"
        )
    }

    private fun vibratePattern(
        timings: LongArray,
        amplitudes: IntArray
    ) {
        val vibrator =
            getSystemService(Context.VIBRATOR_SERVICE) as Vibrator

        if (Build.VERSION.SDK_INT >= 26) {
            vibrator.vibrate(
                VibrationEffect.createWaveform(
                    timings,
                    amplitudes,
                    -1
                )
            )
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
                ) {
                    return
                }

                lastRotationProcessNs = event.timestamp
                rotationProcessedCount++

                try {
                    SensorManager.getRotationMatrixFromVector(
                        rotationMatrix,
                        event.values
                    )
                    rotationReady = true
                } catch (_: Exception) {
                    rotationReady = false
                }
            }

            Sensor.TYPE_LINEAR_ACCELERATION -> {
                if (
                    lastLinearProcessNs > 0L &&
                    event.timestamp - lastLinearProcessNs < SENSOR_PROCESS_PERIOD_NS
                ) {
                    return
                }

                lastLinearProcessNs = event.timestamp
                linearProcessedCount++
                updateForwardAcceleration(event)
            }

            Sensor.TYPE_GYROSCOPE -> {
                if (
                    lastGyroProcessNs > 0L &&
                    event.timestamp - lastGyroProcessNs < SENSOR_PROCESS_PERIOD_NS
                ) {
                    return
                }

                lastGyroProcessNs = event.timestamp
                gyroProcessedCount++

                val gx = event.values[0]
                val gy = event.values[1]
                val gz = event.values[2]

                latestGyroMagnitude =
                    sqrt(
                        (
                            gx * gx +
                                gy * gy +
                                gz * gz
                            ).toDouble()
                    ).toFloat()
            }

            Sensor.TYPE_ACCELEROMETER -> {
                if (
                    lastAccelProcessNs > 0L &&
                    event.timestamp - lastAccelProcessNs < SENSOR_PROCESS_PERIOD_NS
                ) {
                    return
                }

                lastAccelProcessNs = event.timestamp
                accelProcessedCount++

                if (!loggingStarted) return

                val x = event.values[0]
                val y = event.values[1]
                val z = event.values[2]

                val magnitude =
                    sqrt(
                        (
                            x * x +
                                y * y +
                                z * z
                            ).toDouble()
                    ).toFloat()

                val shock =
                    abs(
                        magnitude -
                            SensorManager.GRAVITY_EARTH
                    )

                val now =
                    SystemClock.elapsedRealtime()

                if (
                    latestSpeedKmh >= 8f &&
                    shock >= ROAD_IMPACT_SHOCK_MPS2 &&
                    now - lastRoadImpactElapsed >= ROAD_IMPACT_COOLDOWN_MS
                ) {
                    lastRoadImpactElapsed = now

                    val severity =
                        when {
                            shock >= 7.0f -> "severe"
                            shock >= 5.0f -> "strong"
                            shock >= 4.0f -> "medium"
                            else -> "light"
                        }

                    appendRow(
                        "ROAD_IMPACT_CANDIDATE",
                        x, y, z, magnitude,
                        event.timestamp,
                        !gpsStale,
                        "shock=${String.format(Locale.US, "%.2f", shock)};" +
                            "gyroMag=${String.format(Locale.US, "%.3f", latestGyroMagnitude)};" +
                            "speed=${String.format(Locale.US, "%.1f", latestSpeedKmh)};" +
                            "severity=$severity",
                        null
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

        appendMarker(
            "SENSOR_HEALTH",
            "accelHz=${String.format(Locale.US, "%.1f", accelHz)};" +
                "gyroHz=${String.format(Locale.US, "%.1f", gyroHz)};" +
                "linearHz=${String.format(Locale.US, "%.1f", linearHz)};" +
                "rotationHz=${String.format(Locale.US, "%.1f", rotationHz)};" +
                "rawSensorCsv=false"
        )

        accelProcessedCount = 0
        gyroProcessedCount = 0
        linearProcessedCount = 0
        rotationProcessedCount = 0
        lastSensorHealthElapsed = now
    }

    override fun onAccuracyChanged(
        sensor: Sensor?,
        accuracy: Int
    ) = Unit

    @Synchronized
    private fun appendMarker(
        marker: String,
        details: String = ""
    ) {
        if (!::logFile.isInitialized) return

        appendRow(
            marker,
            Float.NaN, Float.NaN, Float.NaN, Float.NaN,
            0L,
            !gpsStale,
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
        speedValid: Boolean,
        details: String,
        cameraGeometry: CameraGeometry?
    ) {
        if (!::logFile.isInitialized) return

        val wall = System.currentTimeMillis()
        val iso =
            SimpleDateFormat(
                "yyyy-MM-dd'T'HH:mm:ss.SSS",
                Locale.US
            ).format(Date(wall))

        val lat =
            if (latestLat.isNaN()) ""
            else String.format(Locale.US, "%.7f", latestLat)

        val lon =
            if (latestLon.isNaN()) ""
            else String.format(Locale.US, "%.7f", latestLon)

        val gpsAge = currentGpsAgeMs()

        val rawSpeed =
            if (latestRawSpeedKmh.isNaN()) ""
            else String.format(Locale.US, "%.3f", latestRawSpeedKmh)

        val gpsFiltered =
            if (latestGpsFilteredKmh.isNaN()) ""
            else String.format(Locale.US, "%.3f", latestGpsFilteredKmh)

        val fusedSpeed =
            String.format(Locale.US, "%.3f", latestSpeedKmh)

        val forwardAccel =
            String.format(Locale.US, "%.4f", latestForwardAccelMps2)

        val bearing =
            if (latestBearingValid) {
                String.format(Locale.US, "%.3f", latestBearing)
            } else {
                ""
            }

        val cg = cameraGeometry
        val camera = cg?.cluster?.camera

        val cameraId =
            if (cg != null) {
                cg.cluster.memberIds.joinToString("|")
            } else {
                activeCameraId
            }

        val cameraType =
            camera?.type ?: activeCameraType

        val cameraLat =
            camera?.latitude?.let {
                String.format(Locale.US, "%.7f", it)
            } ?: ""

        val cameraLon =
            camera?.longitude?.let {
                String.format(Locale.US, "%.7f", it)
            } ?: ""

        val directDistance =
            cg?.directDistanceM?.let {
                String.format(Locale.US, "%.2f", it)
            } ?: ""

        val vehicleForward =
            cg?.vehicleForwardM?.let {
                String.format(Locale.US, "%.2f", it)
            } ?: ""

        val vehicleCross =
            cg?.vehicleCrossM?.let {
                String.format(Locale.US, "%.2f", it)
            } ?: ""

        val roadCross =
            cg?.roadCrossM?.let {
                String.format(Locale.US, "%.2f", it)
            } ?: ""

        val bearingTo =
            cg?.bearingToCameraDeg?.let {
                String.format(Locale.US, "%.2f", it)
            } ?: ""

        val forwardDelta =
            cg?.forwardDeltaDeg?.let {
                String.format(Locale.US, "%.2f", it)
            } ?: ""

        val axisDelta =
            cg?.roadAxisDeltaDeg?.let {
                String.format(Locale.US, "%.2f", it)
            } ?: ""

        val matchScore =
            cg?.matchScore?.toString() ?: ""

        val requiredScore =
            cg?.requiredScore?.toString() ?: ""

        val confirmed =
            if (cg != null) {
                (
                    cameraStates[cg.cluster.key]?.phase ==
                        TrackPhase.CONFIRMED
                    ).toString()
            } else {
                activeCameraId.isNotEmpty().toString()
            }

        val alertLevel =
            if (cg != null) {
                cameraStates[cg.cluster.key]?.alertLevel?.toString() ?: "0"
            } else {
                activeCameraAlertLevel.toString()
            }

        val ttc =
            cg?.ttcS
                ?.takeIf { it >= 0f }
                ?.let { String.format(Locale.US, "%.2f", it) }
                ?: ""

        val warningDistance =
            cg?.warningDistanceM?.let {
                String.format(Locale.US, "%.1f", it)
            } ?: ""

        val cameraLimit =
            cg?.candidateLimit?.toString()
                ?: activeCameraSpeedLimit?.toString()
                ?: ""

        val sx =
            if (x.isNaN()) ""
            else String.format(Locale.US, "%.6f", x)

        val sy =
            if (y.isNaN()) ""
            else String.format(Locale.US, "%.6f", y)

        val sz =
            if (z.isNaN()) ""
            else String.format(Locale.US, "%.6f", z)

        val sm =
            if (magnitude.isNaN()) ""
            else String.format(Locale.US, "%.6f", magnitude)

        val row =
            listOf(
                wall.toString(),
                iso,
                recordType,
                lat,
                lon,
                latestProvider,
                gpsAge.toString(),
                gpsStale.toString(),
                speedValid.toString(),
                rawSpeed,
                gpsFiltered,
                fusedSpeed,
                forwardAccel,
                fusionActive.toString(),
                imuBridgeActive.toString(),
                bearing,
                latestBearingValid.toString(),
                String.format(Locale.US, "%.2f", latestAccuracy),

                cameraId,
                cameraType,
                cameraLat,
                cameraLon,
                directDistance,
                vehicleForward,
                vehicleCross,
                roadCross,
                bearingTo,
                forwardDelta,
                axisDelta,
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
            ).joinToString(",") {
                csvEscape(it)
            }

        val line = row + "\n"

        try {
            logFile.appendText(line)
        } catch (_: Exception) {}

        try {
            liveWriter?.write(line)
            val now = SystemClock.elapsedRealtime()

            if (
                now - lastLiveFlushElapsed >=
                LIVE_LOG_FLUSH_MS
            ) {
                liveWriter?.flush()
                lastLiveFlushElapsed = now
            }
        } catch (_: Exception) {}
    }

    private fun csvEscape(s: String): String {
        if (
            !s.contains(',') &&
            !s.contains('"') &&
            !s.contains('\n')
        ) {
            return s
        }

        return "\"" +
            s.replace("\"", "\"\"") +
            "\""
    }

    private fun updatePrefs() {
        val age = currentGpsAgeMs()

        prefs.edit()
            .putBoolean("running", true)
            .putFloat("speed_kmh", latestSpeedKmh)
            .putFloat(
                "raw_speed_kmh",
                if (latestRawSpeedKmh.isNaN()) -1f else latestRawSpeedKmh
            )
            .putFloat(
                "gps_filtered_kmh",
                if (latestGpsFilteredKmh.isNaN()) -1f else latestGpsFilteredKmh
            )
            .putFloat("forward_accel_mps2", latestForwardAccelMps2)
            .putBoolean("fusion_active", fusionActive)
            .putBoolean("imu_bridge", imuBridgeActive)
            .putString(
                "lat",
                if (latestLat.isNaN()) null
                else String.format(Locale.US, "%.6f", latestLat)
            )
            .putString(
                "lon",
                if (latestLon.isNaN()) null
                else String.format(Locale.US, "%.6f", latestLon)
            )
            .putFloat("accuracy_m", latestAccuracy)
            .putString("provider", latestProvider)
            .putLong("gps_age_ms", age)
            .putBoolean("gps_stale", gpsStale)
            .putInt("active_limit_kmh", activeLimitKmh)
            .putString("camera_id", activeCameraId)
            .putString("camera_type", activeCameraType)
            .putFloat("camera_distance_m", activeCameraDistanceM)
            .putInt("camera_limit_kmh", activeCameraSpeedLimit ?: -1)
            .putString("camera_road_name", activeCameraRoadName)
            .putFloat("camera_road_cross_m", activeCameraRoadCrossM)
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
            if (
                activeCameraId.isNotEmpty() &&
                activeCameraDistanceM >= 0f
            ) {
                " • camera ${activeCameraDistanceM.roundToInt()}m L$activeCameraAlertLevel"
            } else {
                ""
            }

        val gpsPart =
            when {
                gpsStale -> " • GPS stale"
                imuBridgeActive -> " • IMU bridge"
                fusionActive -> " • GPS+IMU"
                else -> ""
            }

        val text =
            String.format(
                Locale.US,
                "%.0f km/h • limit %d%s%s",
                latestSpeedKmh,
                activeLimitKmh,
                cameraPart,
                gpsPart
            )

        val nm =
            getSystemService(Context.NOTIFICATION_SERVICE) as NotificationManager

        nm.notify(
            NOTIFICATION_ID,
            makeNotification(text)
        )
    }

    private fun makeNotification(
        text: String
    ): Notification {
        val openApp =
            Intent(this, MainActivity::class.java)

        val pending =
            PendingIntent.getActivity(
                this,
                0,
                openApp,
                PendingIntent.FLAG_UPDATE_CURRENT or
                    PendingIntent.FLAG_IMMUTABLE
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
            val channel =
                NotificationChannel(
                    CHANNEL_ID,
                    "Driving tracking",
                    NotificationManager.IMPORTANCE_LOW
                ).apply {
                    description =
                        "Keeps MOJO Drive active while the phone is locked."
                }

            (
                getSystemService(Context.NOTIFICATION_SERVICE) as NotificationManager
                ).createNotificationChannel(channel)
        }
    }

    override fun onProviderEnabled(provider: String) = Unit
    override fun onProviderDisabled(provider: String) = Unit

    override fun onDestroy() {
        handler.removeCallbacks(staleTicker)

        try {
            locationManager.removeUpdates(this)
        } catch (_: Exception) {}

        try {
            sensorManager.unregisterListener(this)
        } catch (_: Exception) {}

        if (
            loggingStarted &&
            !userStopRequested
        ) {
            appendMarker(
                "SERVICE_INTERRUPTED",
                "trip_will_be_recovered=true"
            )

            try {
                liveWriter?.flush()
            } catch (_: Exception) {}

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

        if (userStopRequested) {
            prefs.edit().putBoolean("running", false).apply()
        }

        super.onDestroy()
    }

    override fun onBind(intent: Intent?): IBinder? = null
}
