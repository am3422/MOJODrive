package com.mojotech.mojodrive

import android.Manifest
import android.app.*
import android.content.Context
import android.content.Intent
import android.content.pm.PackageManager
import android.location.Location
import android.location.LocationListener
import android.location.LocationManager
import android.media.AudioManager
import android.media.ToneGenerator
import android.os.Build
import android.os.IBinder
import android.os.VibrationEffect
import android.os.Vibrator
import java.io.File
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

class LocationService : Service(), LocationListener {

    companion object {
        const val EXTRA_THRESHOLD_KMH = "threshold_kmh"
        private const val CHANNEL_ID = "mojo_drive_tracking"
        private const val NOTIFICATION_ID = 100
    }

    private lateinit var locationManager: LocationManager
    private lateinit var prefs: android.content.SharedPreferences
    private var thresholdKmh = 80
    private var overspeedArmed = true
    private var tone: ToneGenerator? = null
    private lateinit var logFile: File

    override fun onCreate() {
        super.onCreate()
        prefs = getSharedPreferences("mojo_drive", MODE_PRIVATE)
        locationManager = getSystemService(Context.LOCATION_SERVICE) as LocationManager
        tone = ToneGenerator(AudioManager.STREAM_NOTIFICATION, 90)
        logFile = File(filesDir, "mojo_drive_trip.csv")
        ensureLogHeader()
        createNotificationChannel()
    }

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        thresholdKmh = intent?.getIntExtra(EXTRA_THRESHOLD_KMH, prefs.getInt("threshold_kmh", 80))
            ?: prefs.getInt("threshold_kmh", 80)

        startForeground(NOTIFICATION_ID, makeNotification("Waiting for GPS…"))
        prefs.edit().putBoolean("running", true).apply()
        startLocationUpdates()
        return START_STICKY
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

    override fun onLocationChanged(location: Location) {
        val speedKmh = if (location.hasSpeed()) location.speed * 3.6f else 0f

        prefs.edit()
            .putBoolean("running", true)
            .putFloat("speed_kmh", speedKmh)
            .putString("lat", String.format(Locale.US, "%.6f", location.latitude))
            .putString("lon", String.format(Locale.US, "%.6f", location.longitude))
            .putFloat("accuracy_m", if (location.hasAccuracy()) location.accuracy else -1f)
            .putString("provider", location.provider ?: "location")
            .apply()

        appendLog(location, speedKmh)
        handleOverspeed(speedKmh)

        val text = String.format(
            Locale.US,
            "%.0f km/h • limit %d",
            speedKmh,
            thresholdKmh
        )
        val nm = getSystemService(Context.NOTIFICATION_SERVICE) as NotificationManager
        nm.notify(NOTIFICATION_ID, makeNotification(text))
    }

    private fun handleOverspeed(speedKmh: Float) {
        if (overspeedArmed && speedKmh >= thresholdKmh) {
            overspeedArmed = false
            tone?.startTone(ToneGenerator.TONE_PROP_BEEP2, 350)
            vibrateAlert()
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

    private fun appendLog(location: Location, speedKmh: Float) {
        try {
            val timestamp = SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS", Locale.US).format(Date())
            val bearing = if (location.hasBearing()) location.bearing else -1f
            val accuracy = if (location.hasAccuracy()) location.accuracy else -1f
            logFile.appendText(
                String.format(
                    Locale.US,
                    "%s,%.7f,%.7f,%.2f,%.2f,%.2f,%s\n",
                    timestamp,
                    location.latitude,
                    location.longitude,
                    speedKmh,
                    bearing,
                    accuracy,
                    location.provider ?: ""
                )
            )
        } catch (_: Exception) {}
    }

    private fun ensureLogHeader() {
        if (!logFile.exists() || logFile.length() == 0L) {
            logFile.writeText("timestamp,lat,lon,speed_kmh,bearing_deg,accuracy_m,provider\n")
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
    @Deprecated("Deprecated in Android API")
    override fun onStatusChanged(provider: String?, status: Int, extras: Bundle?) = Unit

    override fun onDestroy() {
        try {
            locationManager.removeUpdates(this)
        } catch (_: Exception) {}
        tone?.release()
        prefs.edit().putBoolean("running", false).apply()
        super.onDestroy()
    }

    override fun onBind(intent: Intent?): IBinder? = null
}
