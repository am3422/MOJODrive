package com.mojotech.mojodrive

import android.Manifest
import android.app.Activity
import android.content.Intent
import android.content.pm.PackageManager
import android.graphics.Color
import android.os.Build
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.provider.Settings
import android.view.Gravity
import android.view.ViewGroup
import android.widget.*
import java.util.Locale

class MainActivity : Activity() {
    private lateinit var speedText: TextView
    private lateinit var limitText: TextView
    private lateinit var cameraText: TextView
    private lateinit var gpsText: TextView
    private lateinit var statusText: TextView
    private lateinit var coordsText: TextView
    private lateinit var sensorText: TextView
    private lateinit var logText: TextView
    private lateinit var thresholdInput: EditText

    private val handler = Handler(Looper.getMainLooper())
    private val prefs by lazy { getSharedPreferences("mojo_drive", MODE_PRIVATE) }

    private val poller = object : Runnable {
        override fun run() {
            refreshUi()
            handler.postDelayed(this, 400L)
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        title = "MOJO Drive"
        buildUi()
        requestNeededPermissions()
    }

    override fun onResume() {
        super.onResume()
        handler.post(poller)
    }

    override fun onPause() {
        super.onPause()
        handler.removeCallbacks(poller)
    }

    private fun buildUi() {
        val d = resources.displayMetrics.density
        fun dp(v: Int) = (v * d).toInt()

        val root = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            setPadding(dp(22), dp(24), dp(22), dp(24))
            setBackgroundColor(Color.rgb(245, 247, 250))
        }

        root.addView(TextView(this).apply {
            text = "MOJO Drive"
            textSize = 30f
            setTextColor(Color.rgb(25, 30, 38))
            typeface = android.graphics.Typeface.DEFAULT_BOLD
        })

        root.addView(TextView(this).apply {
            text = "MVP 0.9 • Clustered Confidence Camera Engine"
            textSize = 14f
            setTextColor(Color.DKGRAY)
            setPadding(0, dp(4), 0, dp(12))
        })

        speedText = TextView(this).apply {
            text = "0 km/h"
            textSize = 52f
            gravity = Gravity.CENTER
            setTextColor(Color.rgb(20, 91, 210))
            typeface = android.graphics.Typeface.DEFAULT_BOLD
        }
        root.addView(speedText)

        limitText = TextView(this).apply {
            text = "LIMIT 80 km/h"
            textSize = 25f
            gravity = Gravity.CENTER
            typeface = android.graphics.Typeface.DEFAULT_BOLD
        }
        root.addView(limitText)

        cameraText = TextView(this).apply {
            text = "Confirmed camera: --"
            textSize = 17f
            gravity = Gravity.CENTER
            setTextColor(Color.rgb(90, 70, 20))
            setPadding(0, dp(8), 0, dp(8))
        }
        root.addView(cameraText)

        gpsText = TextView(this).apply {
            text = "GPS: waiting"
            textSize = 16f
            gravity = Gravity.CENTER
        }
        root.addView(gpsText)

        statusText = TextView(this).apply {
            text = "Stopped"
            textSize = 15f
            gravity = Gravity.CENTER
            setPadding(0, dp(4), 0, dp(12))
        }
        root.addView(statusText)

        coordsText = TextView(this).apply {
            text = "Location: --"
            textSize = 14f
        }
        root.addView(coordsText)

        sensorText = TextView(this).apply {
            text = "Sensors: --"
            textSize = 14f
            setPadding(0, dp(5), 0, dp(5))
        }
        root.addView(sensorText)

        logText = TextView(this).apply {
            text = "Trip log: --"
            textSize = 13f
            setTextColor(Color.GRAY)
            setPadding(0, dp(5), 0, dp(14))
        }
        root.addView(logText)

        root.addView(TextView(this).apply {
            text = "Fallback speed limit outside camera zones (km/h)"
            textSize = 14f
        })

        thresholdInput = EditText(this).apply {
            inputType = android.text.InputType.TYPE_CLASS_NUMBER
            setText(prefs.getInt("threshold_kmh", 80).toString())
            textSize = 20f
        }
        root.addView(thresholdInput)

        fun addButton(text: String, h: Int, click: () -> Unit) {
            root.addView(Button(this).apply {
                this.text = text
                setOnClickListener { click() }
            }, LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT, dp(h)
            ).apply { topMargin = dp(8) })
        }

        addButton("TEST SOUND + VIBRATION", 52) { testAlert() }
        addButton("START DRIVE", 58) { startDrive() }
        addButton("MARK ROAD EVENT", 52) { markEvent() }
        addButton("STOP & FINALIZE LOG", 54) { stopDrive() }
        addButton("OPEN LOCATION SETTINGS", 48) {
            startActivity(Intent(Settings.ACTION_LOCATION_SOURCE_SETTINGS))
        }

        root.addView(TextView(this).apply {
            text =
                "0.9: duplicate cameras are clustered, severe cross-track is a hard reject, " +
                "confidence changes with distance, confirmed cameras cannot thrash, only one global " +
                "camera alert sounds at a time, short GPS dropouts are bridged, and raw ACCEL/GYRO " +
                "rows are no longer saved to CSV."
            textSize = 12f
            setTextColor(Color.GRAY)
            setPadding(0, dp(14), 0, 0)
        })

        setContentView(ScrollView(this).apply { addView(root) })
    }

    private fun requestNeededPermissions() {
        val permissions = mutableListOf(
            Manifest.permission.ACCESS_FINE_LOCATION,
            Manifest.permission.ACCESS_COARSE_LOCATION
        )
        if (Build.VERSION.SDK_INT >= 33) permissions += Manifest.permission.POST_NOTIFICATIONS
        val missing = permissions.filter {
            checkSelfPermission(it) != PackageManager.PERMISSION_GRANTED
        }
        if (missing.isNotEmpty()) requestPermissions(missing.toTypedArray(), 1001)
    }

    private fun testAlert() {
        startService(
            Intent(this, LocationService::class.java)
                .setAction(LocationService.ACTION_TEST_ALERT)
        )
    }

    private fun startDrive() {
        if (checkSelfPermission(Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
            requestNeededPermissions()
            Toast.makeText(this, "Location permission is required.", Toast.LENGTH_LONG).show()
            return
        }

        val threshold = thresholdInput.text.toString().toIntOrNull()?.coerceIn(20, 250) ?: 80
        prefs.edit().putInt("threshold_kmh", threshold).apply()

        startForegroundService(
            Intent(this, LocationService::class.java)
                .putExtra(LocationService.EXTRA_THRESHOLD_KMH, threshold)
        )

        Toast.makeText(this, "MOJO Drive 0.9 started.", Toast.LENGTH_SHORT).show()
    }

    private fun markEvent() {
        if (!prefs.getBoolean("running", false)) {
            Toast.makeText(this, "Start Drive first.", Toast.LENGTH_SHORT).show()
            return
        }
        startService(
            Intent(this, LocationService::class.java)
                .setAction(LocationService.ACTION_MARK_EVENT)
        )
    }

    private fun stopDrive() {
        if (!prefs.getBoolean("running", false) && !prefs.getBoolean("trip_active", false)) {
            Toast.makeText(this, "No active trip.", Toast.LENGTH_SHORT).show()
            return
        }
        startService(
            Intent(this, LocationService::class.java)
                .setAction(LocationService.ACTION_STOP_TRIP)
        )
        Toast.makeText(this, "Finalizing persistent trip log…", Toast.LENGTH_SHORT).show()
        handler.postDelayed({ refreshUi() }, 1200L)
    }

    private fun refreshUi() {
        val running = prefs.getBoolean("running", false)
        val tripActive = prefs.getBoolean("trip_active", false)
        val speed = prefs.getFloat("speed_kmh", 0f)
        val accuracy = prefs.getFloat("accuracy_m", -1f)
        val gpsAge = prefs.getLong("gps_age_ms", -1L)
        val gpsStale = prefs.getBoolean("gps_stale", true)
        val activeLimit = prefs.getInt("active_limit_kmh", prefs.getInt("threshold_kmh", 80))

        speedText.text = String.format(Locale.US, "%.0f km/h", speed)
        speedText.setTextColor(
            if (running && !gpsStale && speed >= activeLimit + 1f) Color.rgb(210, 35, 35)
            else Color.rgb(20, 91, 210)
        )
        limitText.text = "LIMIT $activeLimit km/h"

        val cameraId = prefs.getString("camera_id", "") ?: ""
        val cameraDistance = prefs.getFloat("camera_distance_m", -1f)
        val cameraLimit = prefs.getInt("camera_limit_kmh", -1)
        val cameraRoad = prefs.getString("camera_road_name", "") ?: ""
        val cameraType = prefs.getString("camera_type", "") ?: ""
        val roadCross = prefs.getFloat("camera_road_cross_m", -1f)
        val forwardDelta = prefs.getFloat("camera_forward_delta", -1f)
        val level = prefs.getInt("camera_alert_level", 0)
        val warn = prefs.getFloat("camera_warning_distance_m", -1f)
        val ttc = prefs.getFloat("camera_ttc_s", -1f)

        cameraText.text =
            if (cameraId.isNotEmpty() && cameraDistance >= 0f) {
                val lim = if (cameraLimit > 0) " • $cameraLimit km/h" else ""
                val road = if (cameraRoad.isNotBlank()) " • $cameraRoad" else ""
                "CONFIRMED ${if (cameraType == "red_light") "red-light" else "speed"} camera $cameraId" +
                    " • ${cameraDistance.toInt()}m$lim$road • warn ${warn.toInt()}m" +
                    " • L$level${if (ttc > 0f) " • ${ttc.toInt()}s" else ""}" +
                    " • cross ${roadCross.toInt()}m • fwdΔ ${forwardDelta.toInt()}°"
            } else {
                "Confirmed camera: --"
            }

        gpsText.text =
            when {
                !running -> "GPS: stopped"
                gpsAge < 0 -> "GPS: waiting for valid speed fix"
                gpsStale -> String.format(Locale.US, "GPS: STALE • %.1f s • acc %.1f m", gpsAge / 1000.0, accuracy)
                prefs.getBoolean("imu_bridge", false) -> String.format(Locale.US, "GPS: IMU BRIDGE • %.1f s", gpsAge / 1000.0)
                else -> String.format(Locale.US, "GPS: OK • age %.1f s • acc %.1f m", gpsAge / 1000.0, accuracy)
            }

        val raw = prefs.getInt("camera_count", 0)
        val clusters = prefs.getInt("camera_cluster_count", 0)
        val confirmed = prefs.getInt("confirmed_camera_count", 0)
        val provider = prefs.getString("provider", "--") ?: "--"

        statusText.text =
            when {
                running -> "ACTIVE • $provider • $raw DB / $clusters clusters • $confirmed confirmed • LIVE LOG"
                tripActive -> "Trip interrupted • persistent log waiting for recovery"
                else -> "Stopped"
            }

        val lat = prefs.getString("lat", null)
        val lon = prefs.getString("lon", null)
        coordsText.text = if (lat != null && lon != null) "Location: $lat, $lon" else "Location: --"

        sensorText.text =
            "Sensors: Accel ${if (prefs.getBoolean("accel_available", false)) "OK" else "--"}" +
                " • Gyro ${if (prefs.getBoolean("gyro_available", false)) "OK" else "--"}" +
                " • Linear ${if (prefs.getBoolean("linear_accel_available", false)) "OK" else "--"}" +
                " • Rotation ${if (prefs.getBoolean("rotation_vector_available", false)) "OK" else "--"}"

        val log = prefs.getString("active_log_name", null) ?: prefs.getString("last_log_name", null)
        logText.text = if (log != null) "Trip log: Downloads/MOJODrive/$log" else "Trip log: --"
    }
}
