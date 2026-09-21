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
    private lateinit var statusText: TextView
    private lateinit var coordsText: TextView
    private lateinit var accuracyText: TextView
    private lateinit var sensorText: TextView
    private lateinit var logText: TextView
    private lateinit var thresholdInput: EditText

    private val handler = Handler(Looper.getMainLooper())
    private val prefs by lazy { getSharedPreferences("mojo_drive", MODE_PRIVATE) }

    private val poller = object : Runnable {
        override fun run() {
            refreshUi()
            handler.postDelayed(this, 500)
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
        val density = resources.displayMetrics.density
        fun dp(v: Int) = (v * density).toInt()

        val root = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            setPadding(dp(22), dp(28), dp(22), dp(22))
            setBackgroundColor(Color.rgb(245, 247, 250))
        }

        root.addView(TextView(this).apply {
            text = "MOJO Drive"
            textSize = 30f
            setTextColor(Color.rgb(25, 30, 38))
            typeface = android.graphics.Typeface.DEFAULT_BOLD
        })

        root.addView(TextView(this).apply {
            text = "MVP 0.2 • GPS + Accelerometer + Gyroscope + Trip Log"
            textSize = 14f
            setTextColor(Color.DKGRAY)
            setPadding(0, dp(4), 0, dp(20))
        })

        speedText = TextView(this).apply {
            text = "0 km/h"
            textSize = 44f
            gravity = Gravity.CENTER
            setTextColor(Color.rgb(20, 91, 210))
            typeface = android.graphics.Typeface.DEFAULT_BOLD
            setPadding(0, dp(14), 0, dp(8))
        }
        root.addView(speedText, LinearLayout.LayoutParams(
            ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.WRAP_CONTENT
        ))

        statusText = TextView(this).apply {
            text = "Stopped"
            textSize = 17f
            gravity = Gravity.CENTER
            setTextColor(Color.DKGRAY)
            setPadding(0, 0, 0, dp(18))
        }
        root.addView(statusText)

        coordsText = TextView(this).apply {
            text = "Location: --"
            textSize = 15f
            setTextColor(Color.DKGRAY)
        }
        root.addView(coordsText)

        accuracyText = TextView(this).apply {
            text = "GPS accuracy: --"
            textSize = 15f
            setTextColor(Color.DKGRAY)
            setPadding(0, dp(5), 0, dp(5))
        }
        root.addView(accuracyText)

        sensorText = TextView(this).apply {
            text = "Sensors: --"
            textSize = 15f
            setTextColor(Color.DKGRAY)
            setPadding(0, dp(5), 0, dp(5))
        }
        root.addView(sensorText)

        logText = TextView(this).apply {
            text = "Last log: --"
            textSize = 13f
            setTextColor(Color.GRAY)
            setPadding(0, dp(5), 0, dp(18))
        }
        root.addView(logText)

        root.addView(TextView(this).apply {
            text = "Speed warning threshold (km/h)"
            textSize = 15f
            setTextColor(Color.rgb(25, 30, 38))
        })

        thresholdInput = EditText(this).apply {
            inputType = android.text.InputType.TYPE_CLASS_NUMBER
            setText(prefs.getInt("threshold_kmh", 80).toString())
            textSize = 20f
        }
        root.addView(thresholdInput)

        root.addView(Button(this).apply {
            text = "START DRIVE"
            textSize = 17f
            setOnClickListener { startDrive() }
        }, LinearLayout.LayoutParams(
            ViewGroup.LayoutParams.MATCH_PARENT, dp(58)
        ).apply { topMargin = dp(20) })

        root.addView(Button(this).apply {
            text = "STOP & SAVE LOG"
            textSize = 16f
            setOnClickListener { stopDrive() }
        }, LinearLayout.LayoutParams(
            ViewGroup.LayoutParams.MATCH_PARENT, dp(54)
        ).apply { topMargin = dp(10) })

        root.addView(Button(this).apply {
            text = "OPEN LOCATION SETTINGS"
            setOnClickListener {
                startActivity(Intent(Settings.ACTION_LOCATION_SOURCE_SETTINGS))
            }
        }, LinearLayout.LayoutParams(
            ViewGroup.LayoutParams.MATCH_PARENT, dp(50)
        ).apply { topMargin = dp(10) })

        root.addView(TextView(this).apply {
            text = "Tomorrow test: Start Drive, lock the phone, drive normally, then press STOP & SAVE LOG after parking. The CSV in Downloads/MOJODrive contains GPS, accelerometer and gyroscope data matched by timestamp/location."
            textSize = 13f
            setTextColor(Color.GRAY)
            setPadding(0, dp(18), 0, 0)
        })

        setContentView(ScrollView(this).apply { addView(root) })
    }

    private fun requestNeededPermissions() {
        val permissions = mutableListOf(
            Manifest.permission.ACCESS_FINE_LOCATION,
            Manifest.permission.ACCESS_COARSE_LOCATION
        )
        if (Build.VERSION.SDK_INT >= 33) {
            permissions += Manifest.permission.POST_NOTIFICATIONS
        }
        val missing = permissions.filter {
            checkSelfPermission(it) != PackageManager.PERMISSION_GRANTED
        }
        if (missing.isNotEmpty()) {
            requestPermissions(missing.toTypedArray(), 1001)
        }
    }

    private fun startDrive() {
        if (checkSelfPermission(Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
            requestNeededPermissions()
            Toast.makeText(this, "Location permission is required.", Toast.LENGTH_LONG).show()
            return
        }

        val threshold = thresholdInput.text.toString().toIntOrNull()?.coerceIn(20, 250) ?: 80
        prefs.edit().putInt("threshold_kmh", threshold).apply()

        val intent = Intent(this, LocationService::class.java)
            .putExtra(LocationService.EXTRA_THRESHOLD_KMH, threshold)

        startForegroundService(intent)
        Toast.makeText(this, "MOJO Drive logging started.", Toast.LENGTH_SHORT).show()
        refreshUi()
    }

    private fun stopDrive() {
        stopService(Intent(this, LocationService::class.java))
        prefs.edit()
            .putBoolean("running", false)
            .putFloat("speed_kmh", 0f)
            .apply()
        Toast.makeText(this, "Stopping and saving trip log…", Toast.LENGTH_SHORT).show()
        handler.postDelayed({ refreshUi() }, 1200)
    }

    private fun refreshUi() {
        val running = prefs.getBoolean("running", false)
        val speed = prefs.getFloat("speed_kmh", 0f)
        val lat = prefs.getString("lat", null)
        val lon = prefs.getString("lon", null)
        val accuracy = prefs.getFloat("accuracy_m", -1f)
        val provider = prefs.getString("provider", "--") ?: "--"
        val accel = prefs.getBoolean("accel_available", false)
        val gyro = prefs.getBoolean("gyro_available", false)
        val lastLog = prefs.getString("last_log_name", null)

        speedText.text = String.format(Locale.US, "%.0f km/h", speed)
        statusText.text = if (running) "ACTIVE • $provider • logging" else "Stopped"
        coordsText.text = if (lat != null && lon != null) "Location: $lat, $lon" else "Location: --"
        accuracyText.text = if (accuracy >= 0f)
            String.format(Locale.US, "GPS accuracy: %.1f m", accuracy)
        else "GPS accuracy: --"
        sensorText.text = "Sensors: Accelerometer ${if (accel) "OK" else "--"} • Gyroscope ${if (gyro) "OK" else "--"}"
        logText.text = if (lastLog != null) "Last log: Downloads/MOJODrive/$lastLog" else "Last log: --"
    }
}
