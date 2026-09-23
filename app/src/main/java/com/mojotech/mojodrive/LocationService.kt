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
        private const val FUSION_BRIDGE_MS = 8000L
        private const val FUSION_MIN_SPEED_KMH = 5f
        private const val ROAD_IMPACT_COOLDOWN_MS = 1200L
        private const val ROAD_IMPACT_SHOCK_MPS2 = 3.2f
        private const val OVERSPEED_REPEAT_MS = 7000L
        private const val CAMERA_SEARCH_M = 1800f
        private const val CAMERA_CONFIRM_EXTRA_M = 250f
        private const val CAMERA_CONFIRM_HITS = 3
        private const val CAMERA_REJECT_HITS = 3
        private const val CAMERA_HEADING_MAX_DEG = 32f
        private const val CAMERA_RETENTION_HEADING_DEG = 45f
        private const val DIRECTION_MIN_SPEED_KMH = 8f
        private const val LIVE_LOG_FLUSH_MS = 1000L
        private const val DIAG_NEAR_M = 700f
        private const val DIAG_INTERVAL_MS = 5000L
    }

    private lateinit var locationManager: LocationManager
    private lateinit var sensorManager: SensorManager
    private lateinit var prefs: android.content.SharedPreferences
    private lateinit var powerManager: PowerManager

    private var accelerometer: Sensor? = null
    private var gyroscope: Sensor? = null
    private var linearAcceleration: Sensor? = null
    private var rotationVector: Sensor? = null
    private var wakeLock: PowerManager.WakeLock? = null

    private var toneLow: ToneGenerator? = null
    private var toneMid: ToneGenerator? = null
    private var toneHigh: ToneGenerator? = null

    private var cameras: List<CameraPoint> = emptyList()

    private val handler = Handler(Looper.getMainLooper())
    private val speedSamples = ArrayDeque<Float>()
    private val rotationMatrix = FloatArray(9)
    private val cameraStates = HashMap<String, CameraTrackState>()

    private var rotationReady = false
    private var fusionInitialized = false
    private var fusionSpeedMps = 0f
    private var fusionVariance = 4f
    private var lastFusionSensorNs = 0L
    private var latestGpsFilteredKmh = Float.NaN
    private var latestForwardAccelMps2 = 0f
    private var fusionActive = false
    private var imuBridgeActive = false
    private var lastRoadImpactElapsed = 0L

    private var manualThresholdKmh = 80
    private var activeLimitKmh = 80
    private var lastAlertLimitKmh = 80
    private var overspeedActive = false
    private var lastOverspeedAlertElapsed = 0L

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

    // UI target = best CONFIRMED camera only. Raw candidates are intentionally hidden.
    @Volatile private var activeCameraId = ""
    @Volatile private var activeCameraType = ""
    @Volatile private var activeCameraDistanceM = -1f
    @Volatile private var activeCameraSpeedLimit: Int? = null
    @Volatile private var activeCameraRoadName = ""
    @Volatile private var activeCameraRoadCrossM = -1f
    @Volatile private var activeCameraHeadingDelta = -1f
    @Volatile private var activeCameraAlertLevel = 0
    @Volatile private var activeCameraTtcS = -1f
    @Volatile private var activeCameraWarningDistanceM = -1f
    @Volatile private var confirmedCameraCount = 0

    private data class CameraTrackState(
        var confirmHits: Int = 0,
        var rejectHits: Int = 0,
        var confirmed: Boolean = false,
        var passed: Boolean = false,
        var passedAtElapsed: Long = 0L,
        var lastDistanceM: Float = Float.MAX_VALUE,
        var minDistanceM: Float = Float.MAX_VALUE,
        var lastAlongM: Float = Float.NaN,
        var warned: Boolean = false,
        var urgentLogged: Boolean = false,
        var alertLevel: Int = 0,
        var lastPulseElapsed: Long = 0L,
        var lastDiagElapsed: Long = 0L,
        var lastSeenElapsed: Long = 0L
    )

    private data class CameraGeometry(
        val camera: CameraPoint,
        val distanceM: Float,
        val alongM: Float,
        val crossTrackM: Float,
        val headingDeltaDeg: Float,
        val chosenRoadBearing: Float?,
        val warningDistanceM: Float,
        val ttcS: Float,
        val candidateLimit: Int?,
        val strictCrossLimitM: Float
    )

    private val staleTicker = object : Runnable {
        override fun run() {
            updateGpsStaleState()
            handler.postDelayed(this, 1000L)
        }
    }

    override fun onCreate() {
        super.onCreate()

        prefs = getSharedPreferences("mojo_drive", MODE_PRIVATE)
        locationManager = getSystemService(Context.LOCATION_SERVICE) as LocationManager
        sensorManager = getSystemService(Context.SENSOR_SERVICE) as SensorManager
        powerManager = getSystemService(Context.POWER_SERVICE) as PowerManager

        accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
        gyroscope = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
        linearAcceleration = sensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION)
        rotationVector = sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR)

        toneLow = ToneGenerator(AudioManager.STREAM_ALARM, 55)
        toneMid = ToneGenerator(AudioManager.STREAM_ALARM, 80)
        toneHigh = ToneGenerator(AudioManager.STREAM_ALARM, 100)

        try {
            cameras = CameraDatabase(this).loadAll()
        } catch (_: Exception) {
            cameras = emptyList()
        }

        prefs.edit()
            .putBoolean("accel_available", accelerometer != null)
            .putBoolean("gyro_available", gyroscope != null)
            .putBoolean("linear_accel_available", linearAcceleration != null)
            .putBoolean("rotation_vector_available", rotationVector != null)
            .putInt("camera_count", cameras.size)
            .apply()

        createNotificationChannel()
        acquireWakeLock()
        handler.post(staleTicker)
    }

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        if (intent?.action == ACTION_TEST_ALERT) {
            runStandaloneAlertTest()
            stopSelf(startId)
            return START_NOT_STICKY
        }

        if (intent?.action == ACTION_MARK_EVENT) {
            if (loggingStarted) {
                appendMarker("USER_ROAD_EVENT")
                vibratePattern(longArrayOf(0, 100), intArrayOf(0, 180))
            }
            return START_STICKY
        }

        if (intent?.action == ACTION_STOP_TRIP) {
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

        manualThresholdKmh = intent?.getIntExtra(
            EXTRA_THRESHOLD_KMH,
            prefs.getInt("threshold_kmh", 80)
        ) ?: prefs.getInt("threshold_kmh", 80)

        activeLimitKmh = manualThresholdKmh
        lastAlertLimitKmh = manualThresholdKmh
        userStopRequested = false

        if (!loggingStarted) {
            if (!resumeInterruptedTrip()) startNewTripLog()
        }

        startForeground(NOTIFICATION_ID, makeNotification("Waiting for valid GPS…"))
        prefs.edit().putBoolean("running", true).apply()

        startLocationUpdates()
        startSensorUpdates()
        return START_STICKY
    }

    private fun runStandaloneAlertTest() {
        val vibrator = getSystemService(Context.VIBRATOR_SERVICE) as Vibrator
        val vibratorPresent = try { vibrator.hasVibrator() } catch (_: Exception) { false }

        val toneOk = try {
            toneHigh?.startTone(ToneGenerator.TONE_PROP_BEEP2, 500) ?: false
        } catch (_: Exception) {
            false
        }

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

        prefs.edit()
            .putBoolean("last_test_tone_ok", toneOk)
            .putBoolean("last_test_vibrator_present", vibratorPresent)
            .putBoolean("last_test_vibration_dispatched", vibrationDispatched)
            .apply()

        Toast.makeText(
            this,
            "Alert test: tone command ${if (toneOk) "OK" else "FAILED"} • vibration ${if (vibrationDispatched) "DISPATCHED" else "FAILED"}",
            Toast.LENGTH_LONG
        ).show()
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

    private fun csvHeader(): String =
        "wall_time_ms,iso_time,record_type,lat,lon,provider,gps_age_ms,gps_stale,speed_valid," +
            "raw_speed_kmh,gps_filtered_kmh,fused_speed_kmh,forward_accel_mps2,fusion_active,imu_bridge," +
            "bearing_deg,bearing_valid,gps_accuracy_m,camera_id,camera_type,camera_distance_m,camera_limit_kmh," +
            "camera_cross_track_m,camera_along_m,camera_heading_delta_deg,camera_confirmed,camera_alert_level," +
            "camera_ttc_s,camera_warning_distance_m,active_limit_kmh,x,y,z,magnitude,sensor_timestamp_ns,details\n"

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
        appendMarker("START", "persistent_live_log=true;camera_engine=road_matched_multi_v07")
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
        if (loggingStarted) return
        resumeInterruptedTrip()
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
            prefs.edit()
                .putBoolean("trip_active", false)
                .putBoolean("running", false)
                .apply()
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
        if (checkSelfPermission(Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED &&
            checkSelfPermission(Manifest.permission.ACCESS_COARSE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
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

        val speedAccuracyOk = if (Build.VERSION.SDK_INT >= 26 && location.hasSpeedAccuracy()) {
            location.speedAccuracyMetersPerSecond <= 5.5f
        } else true

        val accepted = location.hasSpeed() &&
            ageMs in 0..3000 &&
            (!location.hasAccuracy() || location.accuracy <= 50f) &&
            speedAccuracyOk &&
            rawSpeed in 0f..250f

        if (accepted) {
            latestGpsFilteredKmh = filteredSpeed(rawSpeed)
            lastValidGpsElapsedMs = SystemClock.elapsedRealtime()
            gpsStale = false
            imuBridgeActive = false

            latestBearingValid = location.hasBearing() &&
                latestGpsFilteredKmh >= DIRECTION_MIN_SPEED_KMH

            if (latestBearingValid) latestBearing = location.bearing

            val gpsSpeedAccuracyMps = if (Build.VERSION.SDK_INT >= 26 && location.hasSpeedAccuracy()) {
                location.speedAccuracyMetersPerSecond.coerceIn(0.3f, 5.5f)
            } else 2.0f

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
                0L, true,
                "fusion=${if (fusionActive) "GPS_IMU" else "GPS"}"
            )
        } else {
            appendRow(
                "GPS_REJECTED",
                Float.NaN, Float.NaN, Float.NaN, Float.NaN,
                0L, false,
                rejectionReason(location, ageMs, speedAccuracyOk)
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
        } else 0L
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

        val dt = (
            (sensorTimestampNs - lastFusionSensorNs) / 1_000_000_000.0f
        ).coerceIn(0f, 0.20f)

        lastFusionSensorNs = sensorTimestampNs
        if (dt <= 0f) return

        val age = currentGpsAgeMs()
        if (age < 0L || age > FUSION_BRIDGE_MS) return

        val a = if (abs(forwardAccelMps2) < 0.08f) {
            0f
        } else {
            forwardAccelMps2.coerceIn(-8f, 8f)
        }

        fusionSpeedMps = (fusionSpeedMps + a * dt).coerceIn(0f, 70f)
        fusionVariance = (
            fusionVariance + (0.7f * 0.7f * dt)
        ).coerceAtMost(25f)

        latestForwardAccelMps2 = a
        latestSpeedKmh = (fusionSpeedMps * 3.6f).coerceIn(0f, 250f)

        imuBridgeActive =
            age > 1200L && latestSpeedKmh >= FUSION_MIN_SPEED_KMH

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
        val forward = (
            east * sin(br) + north * cos(br)
        ).toFloat()

        predictFusion(forward, event.timestamp)
    }

    private fun updateGpsStaleState() {
        val age = currentGpsAgeMs()
        val staleNow = age < 0 || age > GPS_STALE_MS

        if (staleNow != gpsStale) {
            gpsStale = staleNow
            if (gpsStale && loggingStarted) {
                appendMarker("GPS_STALE")
            }
        }

        imuBridgeActive =
            fusionActive &&
            !gpsStale &&
            age > 1200L &&
            latestSpeedKmh >= FUSION_MIN_SPEED_KMH

        updatePrefs()
    }

    private fun currentGpsAgeMs(): Long {
        if (lastValidGpsElapsedMs <= 0L) return -1L
        return (
            SystemClock.elapsedRealtime() - lastValidGpsElapsedMs
        ).coerceAtLeast(0L)
    }

    private fun evaluateAllCameras() {
        if (
            latestLat.isNaN() ||
            latestLon.isNaN() ||
            cameras.isEmpty() ||
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

        val activeGeometries = ArrayList<CameraGeometry>()

        for (camera in cameras) {
            val geometry = buildCameraGeometry(from, camera) ?: continue
            val key = "${camera.type}:${camera.id}"
            val state = cameraStates.getOrPut(key) { CameraTrackState() }

            state.lastSeenElapsed = now
            state.minDistanceM = min(state.minDistanceM, geometry.distanceM)

            if (
                state.passed &&
                geometry.distanceM > 650f &&
                now - state.passedAtElapsed > 60_000L
            ) {
                cameraStates[key] = CameraTrackState()
                continue
            }

            if (state.passed) continue

            val withinConfirmRange =
                geometry.distanceM <= max(
                    700f,
                    geometry.warningDistanceM + CAMERA_CONFIRM_EXTRA_M
                )

            val approaching =
                state.lastDistanceM == Float.MAX_VALUE ||
                geometry.distanceM <= state.lastDistanceM + 8f

            val strictEligible =
                withinConfirmRange &&
                geometry.alongM > 12f &&
                geometry.crossTrackM <= geometry.strictCrossLimitM &&
                geometry.headingDeltaDeg <= CAMERA_HEADING_MAX_DEG &&
                approaching

            if (strictEligible) {
                state.confirmHits++
                state.rejectHits = 0
            } else {
                state.confirmHits = max(0, state.confirmHits - 1)

                if (state.confirmed) {
                    val retain =
                        geometry.alongM > -15f &&
                        geometry.crossTrackM <= max(45f, geometry.strictCrossLimitM + 15f) &&
                        geometry.headingDeltaDeg <= CAMERA_RETENTION_HEADING_DEG

                    if (!retain) {
                        state.rejectHits++
                        if (state.rejectHits >= CAMERA_REJECT_HITS) {
                            appendCameraEvent(
                                "CAMERA_MATCH_LOST",
                                geometry,
                                state,
                                "reason=left_corridor"
                            )
                            state.confirmed = false
                            state.alertLevel = 0
                        }
                    } else {
                        state.rejectHits = 0
                    }
                }
            }

            if (
                !state.confirmed &&
                state.confirmHits >= CAMERA_CONFIRM_HITS
            ) {
                state.confirmed = true
                state.rejectHits = 0
                appendCameraEvent(
                    "CAMERA_MATCH_CONFIRMED",
                    geometry,
                    state,
                    "confirmHits=${state.confirmHits}"
                )
            }

            if (
                !state.confirmed &&
                geometry.distanceM <= DIAG_NEAR_M &&
                now - state.lastDiagElapsed >= DIAG_INTERVAL_MS
            ) {
                state.lastDiagElapsed = now

                val reason = when {
                    geometry.alongM <= 12f -> "not_ahead"
                    geometry.crossTrackM > geometry.strictCrossLimitM -> "cross_track"
                    geometry.headingDeltaDeg > CAMERA_HEADING_MAX_DEG -> "heading"
                    !approaching -> "not_approaching"
                    else -> "waiting_confirm_hits"
                }

                appendCameraEvent(
                    "CAMERA_MATCH_DIAG",
                    geometry,
                    state,
                    "reason=$reason;confirmHits=${state.confirmHits}"
                )
            }

            // Passed detection uses the camera's road tangent, not just radial distance.
            if (state.confirmed && geometry.alongM < -20f) {
                state.rejectHits++

                if (state.rejectHits >= 2) {
                    appendCameraEvent(
                        "CAMERA_PASSED",
                        geometry,
                        state,
                        "warned=${state.warned};minDistance=${String.format(Locale.US, "%.1f", state.minDistanceM)}"
                    )

                    if (!state.warned) {
                        appendCameraEvent(
                            "CAMERA_MISSED_DIAGNOSTIC",
                            geometry,
                            state,
                            "confirmed_but_no_warning=true"
                        )
                    }

                    state.passed = true
                    state.passedAtElapsed = now
                    state.confirmed = false
                    state.alertLevel = 0
                    state.lastDistanceM = geometry.distanceM
                    continue
                }
            }

            if (state.confirmed) {
                activeGeometries += geometry
                updateCameraAlertForState(key, geometry, state)
            }

            state.lastAlongM = geometry.alongM
            state.lastDistanceM = geometry.distanceM
        }

        confirmedCameraCount = activeGeometries.size
        chooseUiCamera(activeGeometries)
    }

    private fun buildCameraGeometry(
        from: Location,
        camera: CameraPoint
    ): CameraGeometry? {
        val to = Location("camera").apply {
            latitude = camera.latitude
            longitude = camera.longitude
        }

        val distance = from.distanceTo(to)
        if (distance > CAMERA_SEARCH_M) return null

        val bearingToCamera = from.bearingTo(to)
        val chosenRoadBearing = chooseRoadBearing(camera)

        val tangentBearing = chosenRoadBearing ?: latestBearing
        val lineDelta = angleDifference(tangentBearing, bearingToCamera)
        val along = (
            distance * cos(Math.toRadians(lineDelta.toDouble()))
        ).toFloat()

        val cross = abs(
            distance * sin(Math.toRadians(lineDelta.toDouble()))
        ).toFloat()

        val headingDelta = angleDifference(
            latestBearing,
            tangentBearing
        )

        val candidateLimit =
            if (camera.type == "speed") validDynamicLimit(camera) else null

        val warningDistance = computeAdaptiveWarningDistanceM(
            latestSpeedKmh,
            candidateLimit,
            camera.type
        )

        val speedMps = max(1.5f, latestSpeedKmh / 3.6f)
        val ttc = if (along > 0f) along / speedMps else -1f

        // GPS accuracy is incorporated, but the corridor stays intentionally tight.
        // At 3 m GPS accuracy this is ~29.5 m. It can never expand beyond 38 m.
        val strictCrossLimit = (
            25f + max(0f, latestAccuracy) * 1.5f
        ).coerceIn(25f, 38f)

        return CameraGeometry(
            camera = camera,
            distanceM = distance,
            alongM = along,
            crossTrackM = cross,
            headingDeltaDeg = headingDelta,
            chosenRoadBearing = chosenRoadBearing,
            warningDistanceM = warningDistance,
            ttcS = ttc,
            candidateLimit = candidateLimit,
            strictCrossLimitM = strictCrossLimit
        )
    }

    private fun chooseRoadBearing(camera: CameraPoint): Float? {
        val rb = camera.roadBearing ?: return null

        if (
            camera.oneway &&
            (camera.directionConfidence == "high" ||
             camera.directionConfidence == "medium")
        ) {
            return normalizeBearing(rb)
        }

        val opposite = normalizeBearing(rb + 180f)

        return if (
            angleDifference(latestBearing, rb) <=
            angleDifference(latestBearing, opposite)
        ) {
            normalizeBearing(rb)
        } else {
            opposite
        }
    }

    private fun normalizeBearing(v: Float): Float {
        var x = v % 360f
        if (x < 0f) x += 360f
        return x
    }

    private fun updateCameraAlertForState(
        key: String,
        geometry: CameraGeometry,
        state: CameraTrackState
    ) {
        // No alert after the camera line has been passed.
        if (geometry.alongM <= 0f) return

        val inWarningZone =
            geometry.alongM <= geometry.warningDistanceM

        if (!inWarningZone) {
            state.alertLevel = 0
            return
        }

        val level = computeCameraAlertLevel(
            geometry.alongM,
            geometry.ttcS,
            latestSpeedKmh,
            geometry.candidateLimit,
            geometry.warningDistanceM
        )

        state.alertLevel = level
        state.warned = true

        if (geometry.candidateLimit != null) {
            // Use the closest confirmed camera limit only in its actual warning zone.
            if (
                activeCameraSpeedLimit == null ||
                geometry.alongM < activeCameraDistanceM ||
                activeCameraDistanceM < 0f
            ) {
                activeLimitKmh = geometry.candidateLimit
            }
        }

        if (level >= 3 && !state.urgentLogged) {
            state.urgentLogged = true
            appendCameraEvent(
                "CAMERA_URGENT",
                geometry,
                state,
                "level=$level"
            )
        }

        val now = SystemClock.elapsedRealtime()
        val interval = when (level) {
            1 -> 3000L
            2 -> 1800L
            3 -> 900L
            4 -> 450L
            else -> Long.MAX_VALUE
        }

        if (now - state.lastPulseElapsed >= interval) {
            state.lastPulseElapsed = now
            dispatchCameraAlertOutput(
                key,
                geometry,
                level
            )
        }
    }

    private fun chooseUiCamera(
        geometries: List<CameraGeometry>
    ) {
        if (geometries.isEmpty()) {
            clearUiCamera()
            return
        }

        val best = geometries
            .filter { it.alongM > 0f }
            .minByOrNull {
                if (it.ttcS > 0f) it.ttcS else Float.MAX_VALUE
            } ?: run {
                clearUiCamera()
                return
            }

        val key = "${best.camera.type}:${best.camera.id}"
        val state = cameraStates[key]

        activeCameraId = best.camera.id
        activeCameraType = best.camera.type
        activeCameraDistanceM = best.alongM
        activeCameraSpeedLimit = best.candidateLimit
        activeCameraRoadName = best.camera.roadName
        activeCameraRoadCrossM = best.crossTrackM
        activeCameraHeadingDelta = best.headingDeltaDeg
        activeCameraAlertLevel = state?.alertLevel ?: 0
        activeCameraTtcS = best.ttcS
        activeCameraWarningDistanceM = best.warningDistanceM

        activeLimitKmh =
            if (
                best.alongM <= best.warningDistanceM &&
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
        activeCameraHeadingDelta = -1f
        activeCameraAlertLevel = 0
        activeCameraTtcS = -1f
        activeCameraWarningDistanceM = -1f
        confirmedCameraCount = 0
        activeLimitKmh = manualThresholdKmh
    }

    private fun computeAdaptiveWarningDistanceM(
        speedKmh: Float,
        cameraLimitKmh: Int?,
        cameraType: String
    ): Float {
        val v = max(5f, speedKmh) / 3.6f

        // Earlier than 0.6: 22..30 seconds of lead time.
        val baseLeadS = (
            22f + (speedKmh - 50f).coerceAtLeast(0f) * 0.12f
        ).coerceIn(22f, 30f)

        var warningM = v * baseLeadS

        if (
            cameraLimitKmh != null &&
            speedKmh > cameraLimitKmh
        ) {
            val target = cameraLimitKmh / 3.6f
            val decelDistance = (
                (v * v - target * target) / (2f * 1.6f)
            ).coerceAtLeast(0f)

            val reactionDistance = v * 5f

            warningM = max(
                warningM,
                decelDistance + reactionDistance + 90f
            )
        }

        if (cameraType == "red_light") {
            warningM = max(warningM, v * 24f)
        }

        return warningM.coerceIn(400f, 1200f)
    }

    private fun computeCameraAlertLevel(
        alongM: Float,
        ttcS: Float,
        speedKmh: Float,
        cameraLimitKmh: Int?,
        warningDistanceM: Float
    ): Int {
        val overLimit =
            cameraLimitKmh != null &&
            speedKmh > cameraLimitKmh + 2f

        return when {
            (ttcS in 0f..6f || alongM <= 140f) && overLimit -> 4
            ttcS in 0f..9f || alongM <= 220f -> 3
            ttcS in 0f..15f || alongM <= min(400f, warningDistanceM * 0.72f) -> 2
            else -> 1
        }
    }

    private fun dispatchCameraAlertOutput(
        key: String,
        geometry: CameraGeometry,
        level: Int
    ) {
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
            if (geometry.camera.type == "red_light") {
                ToneGenerator.TONE_PROP_ACK
            } else {
                ToneGenerator.TONE_PROP_BEEP2
            }

        val toneOk = try {
            tone?.startTone(toneId, duration) ?: false
        } catch (_: Exception) {
            false
        }

        val vibrator =
            getSystemService(Context.VIBRATOR_SERVICE) as Vibrator

        val vibratorPresent = try {
            vibrator.hasVibrator()
        } catch (_: Exception) {
            false
        }

        var vibrationDispatched = false

        if (vibratorPresent) {
            try {
                when (level) {
                    1 -> vibratePattern(
                        longArrayOf(0, 70),
                        intArrayOf(0, 120)
                    )
                    2 -> vibratePattern(
                        longArrayOf(0, 110),
                        intArrayOf(0, 180)
                    )
                    3 -> vibratePattern(
                        longArrayOf(0, 140),
                        intArrayOf(0, 230)
                    )
                    else -> vibratePattern(
                        longArrayOf(0, 200, 70, 200),
                        intArrayOf(0, 255, 0, 255)
                    )
                }
                vibrationDispatched = true
            } catch (_: Exception) {}
        }

        appendCameraEvent(
            "ALERT_OUTPUT",
            geometry,
            cameraStates[key] ?: CameraTrackState(),
            "level=$level;tone_requested=true;tone_start_ok=$toneOk;vibrator_present=$vibratorPresent;vibration_dispatched=$vibrationDispatched;stream=ALARM;duration_ms=$duration"
        )
    }

    private fun validDynamicLimit(
        camera: CameraPoint
    ): Int? {
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
        appendMarker(
            event,
            "id=${geometry.camera.id};type=${geometry.camera.type};distance=${String.format(Locale.US, "%.1f", geometry.distanceM)};" +
                "along=${String.format(Locale.US, "%.1f", geometry.alongM)};" +
                "cross=${String.format(Locale.US, "%.1f", geometry.crossTrackM)};" +
                "crossLimit=${String.format(Locale.US, "%.1f", geometry.strictCrossLimitM)};" +
                "headingDelta=${String.format(Locale.US, "%.1f", geometry.headingDeltaDeg)};" +
                "roadBearing=${geometry.chosenRoadBearing ?: -1f};" +
                "confirmed=${state.confirmed};warned=${state.warned};$extra"
        )
    }

    private fun angleDifference(a: Float, b: Float): Float {
        var d = abs(a - b) % 360f
        if (d > 180f) d = 360f - d
        return d
    }

    private fun handleOverspeed(speedKmh: Float) {
        if (gpsStale) return

        if (abs(activeLimitKmh - lastAlertLimitKmh) >= 3) {
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

                // Camera-specific alert output is handled separately.
                if (activeCameraAlertLevel == 0) {
                    overspeedAlert()
                }

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
            toneHigh?.startTone(
                ToneGenerator.TONE_PROP_BEEP2,
                500
            ) ?: false
        } catch (_: Exception) {
            false
        }

        val vibrator =
            getSystemService(Context.VIBRATOR_SERVICE) as Vibrator

        val hasVibrator = try {
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

        appendMarker(
            "OVERSPEED_OUTPUT",
            "tone_start_ok=$toneOk;vibrator_present=$hasVibrator;vibration_dispatched=$vibrationDispatched"
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
                try {
                    SensorManager.getRotationMatrixFromVector(
                        rotationMatrix,
                        event.values
                    )
                    rotationReady = true
                } catch (_: Exception) {
                    rotationReady = false
                }
                return
            }

            Sensor.TYPE_LINEAR_ACCELERATION -> {
                updateForwardAcceleration(event)
                return
            }
        }

        if (!loggingStarted) return

        val type = when (event.sensor.type) {
            Sensor.TYPE_ACCELEROMETER -> "ACCEL"
            Sensor.TYPE_GYROSCOPE -> "GYRO"
            else -> return
        }

        val x = event.values[0]
        val y = event.values[1]
        val z = event.values[2]

        val magnitude = sqrt(
            (x * x + y * y + z * z).toDouble()
        ).toFloat()

        if (event.sensor.type == Sensor.TYPE_ACCELEROMETER) {
            val shock = abs(
                magnitude - SensorManager.GRAVITY_EARTH
            )

            val now = SystemClock.elapsedRealtime()

            if (
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

                appendMarker(
                    "ROAD_IMPACT_CANDIDATE",
                    "shock=${String.format(Locale.US, "%.2f", shock)};" +
                        "speed=${String.format(Locale.US, "%.1f", latestSpeedKmh)};" +
                        "severity=$severity"
                )
            }
        }

        appendRow(
            type,
            x, y, z, magnitude,
            event.timestamp,
            !gpsStale,
            ""
        )
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
            details
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
        details: String
    ) {
        if (!::logFile.isInitialized) return

        val wall = System.currentTimeMillis()
        val iso = SimpleDateFormat(
            "yyyy-MM-dd'T'HH:mm:ss.SSS",
            Locale.US
        ).format(Date(wall))

        val lat = if (latestLat.isNaN()) {
            ""
        } else {
            String.format(Locale.US, "%.7f", latestLat)
        }

        val lon = if (latestLon.isNaN()) {
            ""
        } else {
            String.format(Locale.US, "%.7f", latestLon)
        }

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
            } else ""

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

        val cameraConfirmed = activeCameraId.isNotEmpty()

        val row = listOf(
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
            activeCameraId,
            activeCameraType,
            if (activeCameraDistanceM >= 0f) String.format(Locale.US, "%.2f", activeCameraDistanceM) else "",
            activeCameraSpeedLimit?.toString() ?: "",
            if (activeCameraRoadCrossM >= 0f) String.format(Locale.US, "%.2f", activeCameraRoadCrossM) else "",
            if (activeCameraDistanceM >= 0f) String.format(Locale.US, "%.2f", activeCameraDistanceM) else "",
            if (activeCameraHeadingDelta >= 0f) String.format(Locale.US, "%.2f", activeCameraHeadingDelta) else "",
            cameraConfirmed.toString(),
            activeCameraAlertLevel.toString(),
            if (activeCameraTtcS >= 0f) String.format(Locale.US, "%.2f", activeCameraTtcS) else "",
            if (activeCameraWarningDistanceM >= 0f) String.format(Locale.US, "%.1f", activeCameraWarningDistanceM) else "",
            activeLimitKmh.toString(),
            sx,
            sy,
            sz,
            sm,
            sensorTimestampNs.toString(),
            details
        ).joinToString(",") { csvEscape(it) }

        val line = row + "\n"

        try {
            logFile.appendText(line)
        } catch (_: Exception) {}

        try {
            liveWriter?.write(line)

            val now = SystemClock.elapsedRealtime()

            if (
                now - lastLiveFlushElapsed >= LIVE_LOG_FLUSH_MS
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
        ) return s

        return "\"" + s.replace("\"", "\"\"") + "\""
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
            .putFloat("camera_heading_delta", activeCameraHeadingDelta)
            .putInt("camera_alert_level", activeCameraAlertLevel)
            .putFloat("camera_warning_distance_m", activeCameraWarningDistanceM)
            .putFloat("camera_ttc_s", activeCameraTtcS)
            .putInt("confirmed_camera_count", confirmedCameraCount)
            .apply()
    }

    private fun updateNotification() {
        val cameraPart =
            if (
                activeCameraId.isNotEmpty() &&
                activeCameraDistanceM >= 0f
            ) {
                " • confirmed camera ${activeCameraDistanceM.roundToInt()}m L$activeCameraAlertLevel"
            } else ""

        val gpsPart = when {
            gpsStale -> " • GPS stale"
            imuBridgeActive -> " • IMU bridge"
            fusionActive -> " • GPS+IMU"
            else -> ""
        }

        val text = String.format(
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

    private fun makeNotification(text: String): Notification {
        val openApp = Intent(this, MainActivity::class.java)

        val pending = PendingIntent.getActivity(
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
            val channel = NotificationChannel(
                CHANNEL_ID,
                "Driving tracking",
                NotificationManager.IMPORTANCE_LOW
            ).apply {
                description =
                    "Keeps MOJO Drive active while the phone is locked."
            }

            (
                getSystemService(
                    Context.NOTIFICATION_SERVICE
                ) as NotificationManager
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
