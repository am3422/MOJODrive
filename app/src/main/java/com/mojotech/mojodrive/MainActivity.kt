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
    private lateinit var thresholdInput: EditText
    private lateinit var startButton: Button
    private lateinit var stopButton: Button

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

        val title = TextView(this).apply {
            text = "MOJO Drive"
            textSize = 30f
            setTextColor(Color.rgb(25, 30, 38))
            typeface = android.graphics.Typeface.DEFAULT_BOLD
        }
        root.addView(title)

        val subtitle = TextView(this).apply {
            text = "MVP 0.1 • GPS + Background + Overspeed"
            textSize = 14f
            setTextColor(Color.DKGRAY)
            setPadding(0, dp(4), 0, dp(24))
        }
        root.addView(subtitle)

        speedText = TextView(this).apply {
            text = "0 km/h"
            textSize = 44f
            gravity = Gravity.CENTER
            setTextColor(Color.rgb(20, 91, 210))
            typeface = android.graphics.Typeface.DEFAULT_BOLD
            setPadding(0, dp(18), 0, dp(10))
        }
        root.addView(speedText, LinearLayout.LayoutParams(
            ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.WRAP_CONTENT
        ))

        statusText = TextView(this).apply {
            text = "Stopped"
            textSize = 17f
            gravity = Gravity.CENTER
            setTextColor(Color.DKGRAY)
            setPadding(0, 0, 0, dp(22))
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
            setPadding(0, dp(6), 0, dp(20))
        }
        root.addView(accuracyText)

        val label = TextView(this).apply {
            text = "Speed warning threshold (km/h)"
            textSize = 15f
            setTextColor(Color.rgb(25, 30, 38))
        }
        root.addView(label)

        thresholdInput = EditText(this).apply {
            inputType = android.text.InputType.TYPE_CLASS_NUMBER
            setText(prefs.getInt("threshold_kmh", 80).toString())
            textSize = 20f
        }
        root.addView(thresholdInput)

        startButton = Button(this).apply {
            text = "START DRIVE"
            textSize = 17f
            setOnClickListener { startDrive() }
        }
        root.addView(startButton, LinearLayout.LayoutParams(
            ViewGroup.LayoutParams.MATCH_PARENT, dp(58)
        ).apply { topMargin = dp(22) })

        stopButton = Button(this).apply {
            text = "STOP"
            textSize = 16f
            setOnClickListener { stopDrive() }
        }
        root.addView(stopButton, LinearLayout.LayoutParams(
            ViewGroup.LayoutParams.MATCH_PARENT, dp(54)
        ).apply { topMargin = dp(10) })

        val gpsSettingsButton = Button(this).apply {
            text = "OPEN LOCATION SETTINGS"
            setOnClickListener {
                startActivity(Intent(Settings.ACTION_LOCATION_SOURCE_SETTINGS))
            }
        }
        root.addView(gpsSettingsButton, LinearLayout.LayoutParams(
            ViewGroup.LayoutParams.MATCH_PARENT, dp(50)
        ).apply { topMargin = dp(10) })

        val note = TextView(this).apply {
            text = "Test goal: Start Drive, lock the phone, drive normally, and confirm that speed updates and overspeed alerts continue in the background."
            textSize = 13f
            setTextColor(Color.GRAY)
            setPadding(0, dp(20), 0, 0)
        }
        root.addView(note)

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
        Toast.makeText(this, "MOJO Drive started.", Toast.LENGTH_SHORT).show()
        refreshUi()
    }

    private fun stopDrive() {
        stopService(Intent(this, LocationService::class.java))
        prefs.edit()
            .putBoolean("running", false)
            .putFloat("speed_kmh", 0f)
            .apply()
        refreshUi()
    }

    private fun refreshUi() {
        val running = prefs.getBoolean("running", false)
        val speed = prefs.getFloat("speed_kmh", 0f)
        val lat = prefs.getString("lat", null)
        val lon = prefs.getString("lon", null)
        val accuracy = prefs.getFloat("accuracy_m", -1f)
        val provider = prefs.getString("provider", "--") ?: "--"

        speedText.text = String.format(Locale.US, "%.0f km/h", speed)
        statusText.text = if (running) "ACTIVE • $provider" else "Stopped"
        coordsText.text = if (lat != null && lon != null) "Location: $lat, $lon" else "Location: --"
        accuracyText.text = if (accuracy >= 0f)
            String.format(Locale.US, "GPS accuracy: %.1f m", accuracy)
        else "GPS accuracy: --"
    }
}
