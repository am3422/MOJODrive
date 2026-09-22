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
import android.os.*
import android.provider.MediaStore
import android.widget.Toast
import java.io.File
import java.io.FileInputStream
import java.text.SimpleDateFormat
import java.util.ArrayDeque
import java.util.Date
import java.util.Locale
import kotlin.math.*

class LocationService : Service(), LocationListener, SensorEventListener {

    companion object {
        const val EXTRA_THRESHOLD_KMH = "threshold_kmh"
        const val ACTION_MARK_EVENT = "com.mojotech.mojodrive.MARK_EVENT"
        private const val CHANNEL_ID = "mojo_drive_tracking"
        private const val NOTIFICATION_ID = 100
        private const val GPS_STALE_MS = 8000L
        private const val FUSION_BRIDGE_MS = 8000L
        private const val FUSION_MIN_SPEED_KMH = 5f
        private const val BUMP_CANDIDATE_COOLDOWN_MS = 1200L
        private const val BUMP_CANDIDATE_SHOCK_MPS2 = 3.2f
        private const val OVERSPEED_REPEAT_MS = 7000L
        private const val CAMERA_MAX_SEARCH_M = 1500f
        private const val ROAD_DIRECTION_MAX_DELTA_DEG = 45f
        private const val CAMERA_FORWARD_MAX_DELTA_DEG = 75f
        private const val DIRECTION_MIN_SPEED_KMH = 8f
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
    private var tone: ToneGenerator? = null
    private var cameras: List<CameraPoint> = emptyList()

    private val handler = Handler(Looper.getMainLooper())
    private val speedSamples = ArrayDeque<Float>()
    private val cameraWarned = HashSet<String>()
    private val cameraUrgentWarned = HashSet<String>()
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
    private var lastBumpCandidateElapsed = 0L

    private var manualThresholdKmh = 80
    private var activeLimitKmh = 80
    private var lastAlertLimitKmh = 80
    private var overspeedActive = false
    private var lastOverspeedAlertElapsed = 0L

    private lateinit var logFile: File
    private lateinit var logName: String
    private var loggingStarted = false

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
    @Volatile private var activeCameraConfidence = ""
    @Volatile private var activeCameraBearingDelta = -1f
    @Volatile private var activeCameraRoadBearing = -1f
    @Volatile private var activeCameraDirectionDelta = -1f
    @Volatile private var activeCameraLateralM = -1f

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
        tone = ToneGenerator(AudioManager.STREAM_NOTIFICATION, 100)

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
        if (intent?.action == ACTION_MARK_EVENT) {
            if (loggingStarted) {
                appendMarker("USER_EVENT")
                vibratePattern(longArrayOf(0, 100), intArrayOf(0, 180))
            }
            return START_STICKY
        }

        manualThresholdKmh = intent?.getIntExtra(EXTRA_THRESHOLD_KMH, prefs.getInt("threshold_kmh", 80))
            ?: prefs.getInt("threshold_kmh", 80)
        activeLimitKmh = manualThresholdKmh
        lastAlertLimitKmh = manualThresholdKmh

        if (!loggingStarted) startNewTripLog()

        startForeground(NOTIFICATION_ID, makeNotification("Waiting for valid GPS…"))
        prefs.edit().putBoolean("running", true).apply()

        startLocationUpdates()
        startSensorUpdates()
        return START_STICKY
    }

    private fun acquireWakeLock() {
        try {
            wakeLock = powerManager.newWakeLock(PowerManager.PARTIAL_WAKE_LOCK, "MOJODrive::Tracking").apply {
                setReferenceCounted(false)
                acquire()
            }
        } catch (_: Exception) {}
    }

    private fun startNewTripLog() {
        val stamp = SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(Date())
        logName = "MOJODrive_Trip_$stamp.csv"
        logFile = File(filesDir, logName)
        logFile.writeText(
            "wall_time_ms,iso_time,record_type,lat,lon,provider,gps_age_ms,gps_stale,speed_valid," +
                "raw_speed_kmh,gps_filtered_kmh,fused_speed_kmh,forward_accel_mps2,fusion_active,imu_bridge,bearing_deg,bearing_valid,gps_accuracy_m,camera_id," +
                "camera_type,camera_distance_m,camera_limit_kmh,camera_bearing_delta_deg,camera_road_bearing_deg," +
                "camera_direction_delta_deg,camera_lateral_m,active_limit_kmh,x,y,z,magnitude,sensor_timestamp_ns,details\n"
        )
        appendMarker("START")
        loggingStarted = true
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
        rotationVector?.let { sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME) }
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

            latestBearingValid = location.hasBearing() && latestGpsFilteredKmh >= DIRECTION_MIN_SPEED_KMH
            if (latestBearingValid) latestBearing = location.bearing

            val gpsSpeedAccuracyMps = if (Build.VERSION.SDK_INT >= 26 && location.hasSpeedAccuracy()) {
                location.speedAccuracyMetersPerSecond.coerceIn(0.3f, 5.5f)
            } else 2.0f
            correctFusionWithGps(latestGpsFilteredKmh / 3.6f, gpsSpeedAccuracyMps)
            latestSpeedKmh = (fusionSpeedMps * 3.6f).coerceIn(0f, 250f)

            evaluateCameraTarget()
            handleOverspeed(latestSpeedKmh)
            appendRow("GPS", Float.NaN, Float.NaN, Float.NaN, Float.NaN, 0L, true, "fusion=${if (fusionActive) "GPS_IMU" else "GPS"}")
        } else {
            appendRow("GPS_REJECTED", Float.NaN, Float.NaN, Float.NaN, Float.NaN, 0L, false, rejectionReason(location, ageMs, speedAccuracyOk))
        }

        updatePrefs()
        updateNotification()
    }

    private fun rejectionReason(location: Location, ageMs: Long, speedAccuracyOk: Boolean): String {
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
            ((SystemClock.elapsedRealtimeNanos() - location.elapsedRealtimeNanos) / 1_000_000L).coerceAtLeast(0L)
        } else 0L
    }

    private fun filteredSpeed(raw: Float): Float {
        speedSamples.addLast(raw)
        while (speedSamples.size > 3) speedSamples.removeFirst()
        return speedSamples.sorted()[speedSamples.size / 2]
    }


    private fun correctFusionWithGps(gpsSpeedMps: Float, speedAccuracyMps: Float) {
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

    private fun predictFusion(forwardAccelMps2: Float, sensorTimestampNs: Long) {
        if (!fusionInitialized || !latestBearingValid || !rotationReady) {
            lastFusionSensorNs = sensorTimestampNs
            return
        }

        if (lastFusionSensorNs <= 0L) {
            lastFusionSensorNs = sensorTimestampNs
            return
        }

        val dt = ((sensorTimestampNs - lastFusionSensorNs) / 1_000_000_000.0f).coerceIn(0f, 0.20f)
        lastFusionSensorNs = sensorTimestampNs
        if (dt <= 0f) return

        val age = currentGpsAgeMs()
        if (age < 0L || age > FUSION_BRIDGE_MS) return

        val a = when {
            abs(forwardAccelMps2) < 0.08f -> 0f
            else -> forwardAccelMps2.coerceIn(-8f, 8f)
        }

        fusionSpeedMps = (fusionSpeedMps + a * dt).coerceIn(0f, 70f)
        fusionVariance = (fusionVariance + (0.7f * 0.7f * dt)).coerceAtMost(25f)
        latestForwardAccelMps2 = a
        latestSpeedKmh = (fusionSpeedMps * 3.6f).coerceIn(0f, 250f)
        imuBridgeActive = age > 1200L && latestSpeedKmh >= FUSION_MIN_SPEED_KMH
        fusionActive = true

        if (!gpsStale) handleOverspeed(latestSpeedKmh)
        updatePrefs()
    }

    private fun updateForwardAcceleration(event: SensorEvent) {
        if (!rotationReady || !latestBearingValid || event.values.size < 3) return

        val x = event.values[0]
        val y = event.values[1]
        val z = event.values[2]

        val east = rotationMatrix[0] * x + rotationMatrix[1] * y + rotationMatrix[2] * z
        val north = rotationMatrix[3] * x + rotationMatrix[4] * y + rotationMatrix[5] * z
        val br = Math.toRadians(latestBearing.toDouble())
        val forward = (east * sin(br) + north * cos(br)).toFloat()
        predictFusion(forward, event.timestamp)
    }

    private fun updateGpsStaleState() {
        val age = currentGpsAgeMs()
        val staleNow = age < 0 || age > GPS_STALE_MS
        if (staleNow != gpsStale) {
            gpsStale = staleNow
            if (gpsStale && loggingStarted) appendMarker("GPS_STALE")
        }
        imuBridgeActive = fusionActive && !gpsStale && age > 1200L && latestSpeedKmh >= FUSION_MIN_SPEED_KMH
        updatePrefs()
    }

    private fun currentGpsAgeMs(): Long {
        if (lastValidGpsElapsedMs <= 0L) return -1L
        return (SystemClock.elapsedRealtime() - lastValidGpsElapsedMs).coerceAtLeast(0L)
    }

    private data class CameraMatch(
        val camera: CameraPoint,
        val distanceM: Float,
        val bearingDelta: Float,
        val lateralM: Float,
        val roadDirectionDelta: Float
    )

    private fun evaluateCameraTarget() {
        if (latestLat.isNaN() || latestLon.isNaN() || cameras.isEmpty()) {
            clearCameraTarget()
            return
        }

        var best: CameraMatch? = null
        val from = Location("current").apply {
            latitude = latestLat
            longitude = latestLon
        }

        for (camera in cameras) {
            val to = Location("camera").apply {
                latitude = camera.latitude
                longitude = camera.longitude
            }
            val distance = from.distanceTo(to)
            if (distance > CAMERA_MAX_SEARCH_M) continue

            val bearingToCamera = from.bearingTo(to)
            val forwardDelta = if (latestBearingValid) angleDifference(latestBearing, bearingToCamera) else 999f
            val lateral = if (latestBearingValid) {
                abs(distance * sin(Math.toRadians(forwardDelta.toDouble()))).toFloat()
            } else distance

            val roadDelta = if (latestBearingValid && camera.roadBearing != null) {
                angleDifference(latestBearing, camera.roadBearing)
            } else 999f

            val reliableOneWayDirection = camera.oneway &&
                camera.roadBearing != null &&
                (camera.directionConfidence == "high" || camera.directionConfidence == "medium")

            val qualifies = if (latestBearingValid) {
                val forward = distance * cos(Math.toRadians(forwardDelta.toDouble())).toFloat()
                val lateralLimit = max(70f, min(170f, distance * 0.16f))
                val ahead = forward > 10f && forwardDelta <= CAMERA_FORWARD_MAX_DELTA_DEG
                if (reliableOneWayDirection) {
                    ahead && roadDelta <= ROAD_DIRECTION_MAX_DELTA_DEG
                } else {
                    ahead && lateral <= lateralLimit
                }
            } else {
                distance <= 220f
            }

            if (!qualifies) continue
            if (best == null || distance < best!!.distanceM) {
                best = CameraMatch(camera, distance, forwardDelta, lateral, roadDelta)
            }
        }

        if (best == null) {
            clearCameraTarget()
            return
        }

        val match = best!!
        activeCameraId = match.camera.id
        activeCameraType = match.camera.type
        activeCameraDistanceM = match.distanceM
        activeCameraRoadName = match.camera.roadName
        activeCameraConfidence = match.camera.speedConfidence
        activeCameraBearingDelta = match.bearingDelta
        activeCameraLateralM = match.lateralM
        activeCameraRoadBearing = match.camera.roadBearing ?: -1f
        activeCameraDirectionDelta = if (match.roadDirectionDelta < 999f) match.roadDirectionDelta else -1f

        val speedMps = latestSpeedKmh / 3.6f
        val warningSeconds = if (match.camera.type == "red_light") 18f else 25f
        val urgentSeconds = if (match.camera.type == "red_light") 7f else 10f
        val warningDistance = (speedMps * warningSeconds).coerceIn(350f, 1200f)
        val urgentDistance = (speedMps * urgentSeconds).coerceIn(140f, 450f)

        val candidateLimit = if (match.camera.type == "speed") validDynamicLimit(match.camera) else null
        activeCameraSpeedLimit = if (match.distanceM <= warningDistance) candidateLimit else null
        activeLimitKmh = activeCameraSpeedLimit ?: manualThresholdKmh

        val warningKey = "${match.camera.type}:${match.camera.id}"
        if (match.distanceM <= warningDistance && cameraWarned.add(warningKey)) {
            cameraAlert(false, match.camera.type)
            appendMarker(
                if (match.camera.type == "red_light") "RED_LIGHT_CAMERA_WARNING" else "CAMERA_WARNING",
                "id=${match.camera.id};distance=${match.distanceM.roundToInt()};limit=${activeCameraSpeedLimit ?: -1};roadBearing=${match.camera.roadBearing ?: -1};directionDelta=${activeCameraDirectionDelta}"
            )
        }

        if (match.distanceM <= urgentDistance && cameraUrgentWarned.add(warningKey)) {
            cameraAlert(true, match.camera.type)
            appendMarker(
                if (match.camera.type == "red_light") "RED_LIGHT_CAMERA_URGENT" else "CAMERA_URGENT",
                "id=${match.camera.id};distance=${match.distanceM.roundToInt()};limit=${activeCameraSpeedLimit ?: -1};roadBearing=${match.camera.roadBearing ?: -1};directionDelta=${activeCameraDirectionDelta}"
            )
        }
    }

    private fun validDynamicLimit(camera: CameraPoint): Int? {
        val limit = camera.speedLimit ?: return null
        val confidence = camera.speedConfidence.lowercase(Locale.US)
        if (limit !in 20..150) return null
        if (confidence != "high" && confidence != "medium") return null
        if ((camera.roadDistanceM ?: 999.0) > 30.0) return null
        return limit
    }

    private fun clearCameraTarget() {
        activeCameraId = ""
        activeCameraType = ""
        activeCameraDistanceM = -1f
        activeCameraSpeedLimit = null
        activeCameraRoadName = ""
        activeCameraConfidence = ""
        activeCameraBearingDelta = -1f
        activeCameraRoadBearing = -1f
        activeCameraDirectionDelta = -1f
        activeCameraLateralM = -1f
        activeLimitKmh = manualThresholdKmh
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
            if (!overspeedActive || now - lastOverspeedAlertElapsed >= OVERSPEED_REPEAT_MS) {
                overspeedActive = true
                lastOverspeedAlertElapsed = now
                overspeedAlert()
                appendMarker("OVERSPEED_ALERT", "speed=${speedKmh.roundToInt()};limit=$activeLimitKmh;camera=$activeCameraId")
            }
        } else if (speedKmh <= activeLimitKmh - 4f) {
            overspeedActive = false
        }
    }

    private fun cameraAlert(urgent: Boolean, cameraType: String) {
        val duration = if (cameraType == "red_light") { if (urgent) 1000 else 650 } else { if (urgent) 850 else 500 }
        val toneId = if (cameraType == "red_light") ToneGenerator.TONE_PROP_ACK else ToneGenerator.TONE_PROP_BEEP2
        tone?.startTone(toneId, duration)
        if (urgent) {
            vibratePattern(
                longArrayOf(0, 300, 110, 300, 110, 420),
                intArrayOf(0, 255, 0, 255, 0, 255)
            )
        } else {
            vibratePattern(
                longArrayOf(0, 190, 100, 190),
                intArrayOf(0, 220, 0, 220)
            )
        }
    }

    private fun overspeedAlert() {
        tone?.startTone(ToneGenerator.TONE_PROP_BEEP2, 1000)
        vibratePattern(
            longArrayOf(0, 450, 120, 450, 120, 550),
            intArrayOf(0, 255, 0, 255, 0, 255)
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
                try {
                    SensorManager.getRotationMatrixFromVector(rotationMatrix, event.values)
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
        val magnitude = sqrt((x * x + y * y + z * z).toDouble()).toFloat()

        if (event.sensor.type == Sensor.TYPE_ACCELEROMETER) {
            val shock = abs(magnitude - SensorManager.GRAVITY_EARTH)
            val now = SystemClock.elapsedRealtime()
            if (latestSpeedKmh >= 8f && shock >= BUMP_CANDIDATE_SHOCK_MPS2 && now - lastBumpCandidateElapsed >= BUMP_CANDIDATE_COOLDOWN_MS) {
                lastBumpCandidateElapsed = now
                appendMarker("BUMP_CANDIDATE", "shock=${String.format(Locale.US, "%.2f", shock)};speed=${String.format(Locale.US, "%.1f", latestSpeedKmh)}")
            }
        }

        appendRow(type, x, y, z, magnitude, event.timestamp, !gpsStale, "")
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) = Unit

    @Synchronized
    private fun appendMarker(marker: String, details: String = "") {
        if (!::logFile.isInitialized) return
        appendRow(marker, Float.NaN, Float.NaN, Float.NaN, Float.NaN, 0L, !gpsStale, details)
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
        val iso = SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss.SSS", Locale.US).format(Date(wall))
        val lat = if (latestLat.isNaN()) "" else String.format(Locale.US, "%.7f", latestLat)
        val lon = if (latestLon.isNaN()) "" else String.format(Locale.US, "%.7f", latestLon)
        val gpsAge = currentGpsAgeMs()
        val rawSpeed = if (latestRawSpeedKmh.isNaN()) "" else String.format(Locale.US, "%.3f", latestRawSpeedKmh)
        val gpsFiltered = if (latestGpsFilteredKmh.isNaN()) "" else String.format(Locale.US, "%.3f", latestGpsFilteredKmh)
        val fusedSpeed = String.format(Locale.US, "%.3f", latestSpeedKmh)
        val forwardAccel = String.format(Locale.US, "%.4f", latestForwardAccelMps2)
        val bearing = if (latestBearingValid) String.format(Locale.US, "%.3f", latestBearing) else ""
        val cameraDistance = if (activeCameraDistanceM >= 0f) String.format(Locale.US, "%.2f", activeCameraDistanceM) else ""
        val cameraLimit = activeCameraSpeedLimit?.toString() ?: ""
        val cameraDelta = if (activeCameraBearingDelta >= 0f) String.format(Locale.US, "%.2f", activeCameraBearingDelta) else ""
        val cameraRoadBearing = if (activeCameraRoadBearing >= 0f) String.format(Locale.US, "%.2f", activeCameraRoadBearing) else ""
        val cameraDirectionDelta = if (activeCameraDirectionDelta >= 0f) String.format(Locale.US, "%.2f", activeCameraDirectionDelta) else ""
        val cameraLateral = if (activeCameraLateralM >= 0f) String.format(Locale.US, "%.2f", activeCameraLateralM) else ""
        val sx = if (x.isNaN()) "" else String.format(Locale.US, "%.6f", x)
        val sy = if (y.isNaN()) "" else String.format(Locale.US, "%.6f", y)
        val sz = if (z.isNaN()) "" else String.format(Locale.US, "%.6f", z)
        val sm = if (magnitude.isNaN()) "" else String.format(Locale.US, "%.6f", magnitude)

        val row = listOf(
            wall.toString(), iso, recordType, lat, lon, latestProvider,
            gpsAge.toString(), gpsStale.toString(), speedValid.toString(), rawSpeed,
            gpsFiltered, fusedSpeed, forwardAccel, fusionActive.toString(), imuBridgeActive.toString(), bearing,
            latestBearingValid.toString(), String.format(Locale.US, "%.2f", latestAccuracy),
            activeCameraId, activeCameraType, cameraDistance, cameraLimit, cameraDelta, cameraRoadBearing, cameraDirectionDelta, cameraLateral,
            activeLimitKmh.toString(), sx, sy, sz, sm, sensorTimestampNs.toString(), details
        ).joinToString(",") { csvEscape(it) }

        try { logFile.appendText(row + "\n") } catch (_: Exception) {}
    }

    private fun csvEscape(s: String): String {
        if (!s.contains(',') && !s.contains('"') && !s.contains('\n')) return s
        return "\"" + s.replace("\"", "\"\"") + "\""
    }

    private fun updatePrefs() {
        val age = currentGpsAgeMs()
        prefs.edit()
            .putBoolean("running", true)
            .putFloat("speed_kmh", latestSpeedKmh)
            .putFloat("raw_speed_kmh", if (latestRawSpeedKmh.isNaN()) -1f else latestRawSpeedKmh)
            .putFloat("gps_filtered_kmh", if (latestGpsFilteredKmh.isNaN()) -1f else latestGpsFilteredKmh)
            .putFloat("forward_accel_mps2", latestForwardAccelMps2)
            .putBoolean("fusion_active", fusionActive)
            .putBoolean("imu_bridge", imuBridgeActive)
            .putString("lat", if (latestLat.isNaN()) null else String.format(Locale.US, "%.6f", latestLat))
            .putString("lon", if (latestLon.isNaN()) null else String.format(Locale.US, "%.6f", latestLon))
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
            .putString("camera_confidence", activeCameraConfidence)
            .putFloat("camera_road_bearing", activeCameraRoadBearing)
            .putFloat("camera_direction_delta", activeCameraDirectionDelta)
            .apply()
    }

    private fun updateNotification() {
        val cameraPart = if (activeCameraId.isNotEmpty() && activeCameraDistanceM >= 0f) {
            " • ${if (activeCameraType == "red_light") "red-light" else "camera"} ${activeCameraDistanceM.roundToInt()}m"
        } else ""
        val gpsPart = when {
            gpsStale -> " • GPS stale"
            imuBridgeActive -> " • IMU bridge"
            fusionActive -> " • GPS+IMU"
            else -> ""
        }
        val text = String.format(Locale.US, "%.0f km/h • limit %d%s%s", latestSpeedKmh, activeLimitKmh, cameraPart, gpsPart)
        val nm = getSystemService(Context.NOTIFICATION_SERVICE) as NotificationManager
        nm.notify(NOTIFICATION_ID, makeNotification(text))
    }

    private fun exportLogToDownloads() {
        if (!::logFile.isInitialized || !logFile.exists()) return
        try {
            if (Build.VERSION.SDK_INT >= 29) {
                val values = ContentValues().apply {
                    put(MediaStore.Downloads.DISPLAY_NAME, logName)
                    put(MediaStore.Downloads.MIME_TYPE, "text/csv")
                    put(MediaStore.Downloads.RELATIVE_PATH, Environment.DIRECTORY_DOWNLOADS + "/MOJODrive")
                    put(MediaStore.Downloads.IS_PENDING, 1)
                }
                val resolver = contentResolver
                val uri = resolver.insert(MediaStore.Downloads.EXTERNAL_CONTENT_URI, values)
                if (uri != null) {
                    resolver.openOutputStream(uri)?.use { out ->
                        FileInputStream(logFile).use { input -> input.copyTo(out) }
                    }
                    values.clear()
                    values.put(MediaStore.Downloads.IS_PENDING, 0)
                    resolver.update(uri, values, null, null)
                }
            } else {
                val dir = getExternalFilesDir(Environment.DIRECTORY_DOCUMENTS)
                if (dir != null) {
                    val outDir = File(dir, "MOJODrive").apply { mkdirs() }
                    logFile.copyTo(File(outDir, logName), overwrite = true)
                }
            }
            prefs.edit().putString("last_log_name", logName).apply()
            Toast.makeText(this, "Trip log saved: Downloads/MOJODrive/$logName", Toast.LENGTH_LONG).show()
        } catch (e: Exception) {
            Toast.makeText(this, "Could not export trip log: ${e.message}", Toast.LENGTH_LONG).show()
        }
    }

    private fun makeNotification(text: String): Notification {
        val openApp = Intent(this, MainActivity::class.java)
        val pending = PendingIntent.getActivity(
            this, 0, openApp,
            PendingIntent.FLAG_UPDATE_CURRENT or PendingIntent.FLAG_IMMUTABLE
        )
        return Notification.Builder(this, CHANNEL_ID)
            .setContentTitle("MOJO Drive active")
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
            ).apply { description = "Keeps MOJO Drive active while the phone is locked." }
            (getSystemService(Context.NOTIFICATION_SERVICE) as NotificationManager)
                .createNotificationChannel(channel)
        }
    }

    override fun onProviderEnabled(provider: String) = Unit
    override fun onProviderDisabled(provider: String) = Unit

    override fun onDestroy() {
        handler.removeCallbacks(staleTicker)
        try { locationManager.removeUpdates(this) } catch (_: Exception) {}
        try { sensorManager.unregisterListener(this) } catch (_: Exception) {}

        if (loggingStarted) {
            appendMarker("STOP")
            exportLogToDownloads()
            loggingStarted = false
        }

        try { if (wakeLock?.isHeld == true) wakeLock?.release() } catch (_: Exception) {}
        tone?.release()
        prefs.edit().putBoolean("running", false).apply()
        super.onDestroy()
    }

    override fun onBind(intent: Intent?): IBinder? = null
}
