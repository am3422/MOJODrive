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
import android.os.Build
import android.os.Environment
import android.os.IBinder
import android.os.VibrationEffect
import android.os.Vibrator
import android.provider.MediaStore
import android.widget.Toast
import java.io.File
import java.io.FileInputStream
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import kotlin.math.sqrt

class LocationService : Service(), LocationListener, SensorEventListener {

    companion object {
        const val EXTRA_THRESHOLD_KMH = "threshold_kmh"
        private const val CHANNEL_ID = "mojo_drive_tracking"
        private const val NOTIFICATION_ID = 100
    }

    private lateinit var locationManager: LocationManager
    private lateinit var sensorManager: SensorManager
    private lateinit var prefs: android.content.SharedPreferences

    private var accelerometer: Sensor? = null
    private var gyroscope: Sensor? = null

    private var thresholdKmh = 80
    private var overspeedArmed = true
    private var tone: ToneGenerator? = null
    private lateinit var logFile: File
    private lateinit var logName: String
    private var loggingStarted = false

    @Volatile private var latestLat = Double.NaN
    @Volatile private var latestLon = Double.NaN
    @Volatile private var latestSpeedKmh = 0f
    @Volatile private var latestBearing = -1f
    @Volatile private var latestAccuracy = -1f

    override fun onCreate() {
        super.onCreate()
        prefs = getSharedPreferences("mojo_drive", MODE_PRIVATE)
        locationManager = getSystemService(Context.LOCATION_SERVICE) as LocationManager
        sensorManager = getSystemService(Context.SENSOR_SERVICE) as SensorManager
        accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
        gyroscope = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
        tone = ToneGenerator(AudioManager.STREAM_NOTIFICATION, 90)

        prefs.edit()
            .putBoolean("accel_available", accelerometer != null)
            .putBoolean("gyro_available", gyroscope != null)
            .apply()

        createNotificationChannel()
    }

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        thresholdKmh = intent?.getIntExtra(EXTRA_THRESHOLD_KMH, prefs.getInt("threshold_kmh", 80))
            ?: prefs.getInt("threshold_kmh", 80)

        if (!loggingStarted) startNewTripLog()

        startForeground(NOTIFICATION_ID, makeNotification("Waiting for GPS…"))
        prefs.edit().putBoolean("running", true).apply()

        startLocationUpdates()
        startSensorUpdates()
        return START_STICKY
    }

    private fun startNewTripLog() {
        val stamp = SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(Date())
        logName = "MOJODrive_Trip_$stamp.csv"
        logFile = File(filesDir, logName)
        logFile.writeText(
            "wall_time_ms,iso_time,record_type,lat,lon,speed_kmh,bearing_deg,gps_accuracy_m," +
            "x,y,z,magnitude,sensor_timestamp_ns\n"
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
                    1000L,
                    0f,
                    this
                )
            }
        } catch (_: Exception) {}

        try {
            if (locationManager.isProviderEnabled(LocationManager.NETWORK_PROVIDER)) {
                locationManager.requestLocationUpdates(
                    LocationManager.NETWORK_PROVIDER,
                    2000L,
                    0f,
                    this
                )
            }
        } catch (_: Exception) {}
    }

    private fun startSensorUpdates() {
        // 50,000 us ~= 20 Hz per sensor: enough for feasibility testing without huge logs.
        accelerometer?.let { sensorManager.registerListener(this, it, 50_000) }
        gyroscope?.let { sensorManager.registerListener(this, it, 50_000) }
    }

    override fun onLocationChanged(location: Location) {
        latestLat = location.latitude
        latestLon = location.longitude
        latestSpeedKmh = if (location.hasSpeed()) location.speed * 3.6f else 0f
        latestBearing = if (location.hasBearing()) location.bearing else -1f
        latestAccuracy = if (location.hasAccuracy()) location.accuracy else -1f

        prefs.edit()
            .putBoolean("running", true)
            .putFloat("speed_kmh", latestSpeedKmh)
            .putString("lat", String.format(Locale.US, "%.6f", latestLat))
            .putString("lon", String.format(Locale.US, "%.6f", latestLon))
            .putFloat("accuracy_m", latestAccuracy)
            .putString("provider", location.provider ?: "location")
            .apply()

        appendGpsRow()
        handleOverspeed(latestSpeedKmh)

        val text = String.format(
            Locale.US,
            "%.0f km/h • limit %d • sensors logging",
            latestSpeedKmh,
            thresholdKmh
        )
        val nm = getSystemService(Context.NOTIFICATION_SERVICE) as NotificationManager
        nm.notify(NOTIFICATION_ID, makeNotification(text))
    }

    override fun onSensorChanged(event: SensorEvent) {
        if (!loggingStarted || event.values.size < 3) return

        val type = when (event.sensor.type) {
            Sensor.TYPE_ACCELEROMETER -> "ACCEL"
            Sensor.TYPE_GYROSCOPE -> "GYRO"
            else -> return
        }

        val x = event.values[0]
        val y = event.values[1]
        val z = event.values[2]
        val magnitude = sqrt(x * x + y * y + z * z)

        appendRow(
            recordType = type,
            x = x,
            y = y,
            z = z,
            magnitude = magnitude,
            sensorTimestampNs = event.timestamp
        )
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) = Unit

    private fun appendGpsRow() {
        appendRow(
            recordType = "GPS",
            x = Float.NaN,
            y = Float.NaN,
            z = Float.NaN,
            magnitude = Float.NaN,
            sensorTimestampNs = 0L
        )
    }

    private fun appendMarker(marker: String) {
        if (!::logFile.isInitialized) return
        val wall = System.currentTimeMillis()
        val iso = SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss.SSS", Locale.US).format(Date(wall))
        try {
            logFile.appendText("$wall,$iso,$marker,,,,,,,,,,\n")
        } catch (_: Exception) {}
    }

    private fun appendRow(
        recordType: String,
        x: Float,
        y: Float,
        z: Float,
        magnitude: Float,
        sensorTimestampNs: Long
    ) {
        if (!::logFile.isInitialized) return

        val wall = System.currentTimeMillis()
        val iso = SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss.SSS", Locale.US).format(Date(wall))

        val lat = if (latestLat.isNaN()) "" else String.format(Locale.US, "%.7f", latestLat)
        val lon = if (latestLon.isNaN()) "" else String.format(Locale.US, "%.7f", latestLon)
        val xyz = if (x.isNaN()) ",,,," else String.format(
            Locale.US, "%.6f,%.6f,%.6f,%.6f", x, y, z, magnitude
        )

        try {
            logFile.appendText(
                String.format(
                    Locale.US,
                    "%d,%s,%s,%s,%s,%.3f,%.3f,%.3f,%s,%d\n",
                    wall,
                    iso,
                    recordType,
                    lat,
                    lon,
                    latestSpeedKmh,
                    latestBearing,
                    latestAccuracy,
                    xyz,
                    sensorTimestampNs
                )
            )
        } catch (_: Exception) {}
    }

    private fun handleOverspeed(speedKmh: Float) {
        if (overspeedArmed && speedKmh >= thresholdKmh) {
            overspeedArmed = false
            tone?.startTone(ToneGenerator.TONE_PROP_BEEP2, 350)
            vibrateAlert()
            appendMarker("OVERSPEED_ALERT")
        } else if (!overspeedArmed && speedKmh <= thresholdKmh - 5) {
            overspeedArmed = true
        }
    }

    private fun vibrateAlert() {
        val vibrator = getSystemService(Context.VIBRATOR_SERVICE) as Vibrator
        if (Build.VERSION.SDK_INT >= 26) {
            vibrator.vibrate(
                VibrationEffect.createWaveform(longArrayOf(0, 180, 100, 180), -1)
            )
        } else {
            @Suppress("DEPRECATION")
            vibrator.vibrate(400)
        }
    }

    private fun exportLogToDownloads() {
        if (!::logFile.isInitialized || !logFile.exists()) return
        try {
            if (Build.VERSION.SDK_INT >= 29) {
                val values = ContentValues().apply {
                    put(MediaStore.Downloads.DISPLAY_NAME, logName)
                    put(MediaStore.Downloads.MIME_TYPE, "text/csv")
                    put(
                        MediaStore.Downloads.RELATIVE_PATH,
                        Environment.DIRECTORY_DOWNLOADS + "/MOJODrive"
                    )
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
            Toast.makeText(
                this,
                "Trip log saved: Downloads/MOJODrive/$logName",
                Toast.LENGTH_LONG
            ).show()
        } catch (e: Exception) {
            Toast.makeText(this, "Could not export trip log: ${e.message}", Toast.LENGTH_LONG).show()
        }
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
            ).apply {
                description = "Keeps MOJO Drive active while the phone is locked."
            }
            (getSystemService(Context.NOTIFICATION_SERVICE) as NotificationManager)
                .createNotificationChannel(channel)
        }
    }

    override fun onProviderEnabled(provider: String) = Unit
    override fun onProviderDisabled(provider: String) = Unit

    override fun onDestroy() {
        try { locationManager.removeUpdates(this) } catch (_: Exception) {}
        try { sensorManager.unregisterListener(this) } catch (_: Exception) {}

        if (loggingStarted) {
            appendMarker("STOP")
            exportLogToDownloads()
            loggingStarted = false
        }

        tone?.release()
        prefs.edit().putBoolean("running", false).apply()
        super.onDestroy()
    }

    override fun onBind(intent: Intent?): IBinder? = null
}
