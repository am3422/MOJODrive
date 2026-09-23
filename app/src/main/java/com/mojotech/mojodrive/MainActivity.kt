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
    private val prefs by lazy {
        getSharedPreferences("mojo_drive", MODE_PRIVATE)
    }

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
        val density = resources.displayMetrics.density
        fun dp(v: Int) = (v * density).toInt()

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
            text = "MVP 0.8 • Hybrid GPS + Road-Axis Camera Engine"
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
            setPadding(0, dp(10), 0, dp(2))
        }

        root.addView(
            speedText,
            LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT,
                ViewGroup.LayoutParams.WRAP_CONTENT
            )
        )

        limitText = TextView(this).apply {
            text = "LIMIT 80 km/h"
            textSize = 25f
            gravity = Gravity.CENTER
            setTextColor(Color.rgb(30, 30, 30))
            typeface = android.graphics.Typeface.DEFAULT_BOLD
            setPadding(0, 0, 0, dp(10))
        }
        root.addView(limitText)

        cameraText = TextView(this).apply {
            text = "Confirmed camera: --"
            textSize = 17f
            gravity = Gravity.CENTER
            setTextColor(Color.rgb(90, 70, 20))
            setPadding(0, dp(4), 0, dp(8))
        }
        root.addView(cameraText)

        gpsText = TextView(this).apply {
            text = "GPS: waiting"
            textSize = 16f
            gravity = Gravity.CENTER
            setTextColor(Color.DKGRAY)
            setPadding(0, 0, 0, dp(6))
        }
        root.addView(gpsText)

        statusText = TextView(this).apply {
            text = "Stopped"
            textSize = 15f
            gravity = Gravity.CENTER
            setTextColor(Color.DKGRAY)
            setPadding(0, 0, 0, dp(14))
        }
        root.addView(statusText)

        coordsText = TextView(this).apply {
            text = "Location: --"
            textSize = 14f
            setTextColor(Color.DKGRAY)
        }
        root.addView(coordsText)

        sensorText = TextView(this).apply {
            text = "Sensors: --"
            textSize = 14f
            setTextColor(Color.DKGRAY)
            setPadding(0, dp(5), 0, dp(5))
        }
        root.addView(sensorText)

        logText = TextView(this).apply {
            text = "Trip log: --"
            textSize = 13f
            setTextColor(Color.GRAY)
            setPadding(0, dp(5), 0, dp(16))
        }
        root.addView(logText)

        root.addView(TextView(this).apply {
            text = "Fallback speed limit outside camera zones (km/h)"
            textSize = 14f
            setTextColor(Color.rgb(25, 30, 38))
        })

        thresholdInput = EditText(this).apply {
            inputType = android.text.InputType.TYPE_CLASS_NUMBER
            setText(
                prefs.getInt("threshold_kmh", 80).toString()
            )
            textSize = 20f
        }
        root.addView(thresholdInput)

        root.addView(Button(this).apply {
            text = "TEST SOUND + VIBRATION"
            textSize = 15f
            setOnClickListener { testAlert() }
        }, LinearLayout.LayoutParams(
            ViewGroup.LayoutParams.MATCH_PARENT,
            dp(52)
        ).apply {
            topMargin = dp(16)
        })

        root.addView(Button(this).apply {
            text = "START DRIVE"
            textSize = 17f
            setOnClickListener { startDrive() }
        }, LinearLayout.LayoutParams(
            ViewGroup.LayoutParams.MATCH_PARENT,
            dp(58)
        ).apply {
            topMargin = dp(8)
        })

        root.addView(Button(this).apply {
            text = "MARK ROAD EVENT"
            textSize = 16f
            setOnClickListener { markEvent() }
        }, LinearLayout.LayoutParams(
            ViewGroup.LayoutParams.MATCH_PARENT,
            dp(52)
        ).apply {
            topMargin = dp(8)
        })

        root.addView(Button(this).apply {
            text = "STOP & FINALIZE LOG"
            textSize = 16f
            setOnClickListener { stopDrive() }
        }, LinearLayout.LayoutParams(
            ViewGroup.LayoutParams.MATCH_PARENT,
            dp(54)
        ).apply {
            topMargin = dp(8)
        })

        root.addView(Button(this).apply {
            text = "OPEN LOCATION SETTINGS"
            setOnClickListener {
                startActivity(
                    Intent(Settings.ACTION_LOCATION_SOURCE_SETTINGS)
                )
            }
        }, LinearLayout.LayoutParams(
            ViewGroup.LayoutParams.MATCH_PARENT,
            dp(48)
        ).apply {
            topMargin = dp(8)
        })

        root.addView(TextView(this).apply {
            text =
                "0.8: roadBearing is now used only as a road AXIS, never blindly as forward direction. " +
                "Ahead/behind comes from the phone GPS travel bearing. Multiple cameras are tracked independently. " +
                "False parallel-road matches are rejected by cross-track distance, while approach history prevents real cameras being dropped. " +
                "Every confirmation, rejection reason, pass and alert output is logged."
            textSize = 12f
            setTextColor(Color.GRAY)
            setPadding(0, dp(16), 0, 0)
        })

        setContentView(
            ScrollView(this).apply {
                addView(root)
            }
        )
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
            checkSelfPermission(it) !=
                PackageManager.PERMISSION_GRANTED
        }

        if (missing.isNotEmpty()) {
            requestPermissions(
                missing.toTypedArray(),
                1001
            )
        }
    }

    private fun testAlert() {
        startService(
            Intent(
                this,
                LocationService::class.java
            ).setAction(
                LocationService.ACTION_TEST_ALERT
            )
        )
    }

    private fun startDrive() {
        if (
            checkSelfPermission(
                Manifest.permission.ACCESS_FINE_LOCATION
            ) != PackageManager.PERMISSION_GRANTED
        ) {
            requestNeededPermissions()

            Toast.makeText(
                this,
                "Location permission is required.",
                Toast.LENGTH_LONG
            ).show()

            return
        }

        val threshold =
            thresholdInput.text.toString()
                .toIntOrNull()
                ?.coerceIn(20, 250)
                ?: 80

        prefs.edit()
            .putInt("threshold_kmh", threshold)
            .apply()

        val intent = Intent(
            this,
            LocationService::class.java
        ).putExtra(
            LocationService.EXTRA_THRESHOLD_KMH,
            threshold
        )

        startForegroundService(intent)

        Toast.makeText(
            this,
            "MOJO Drive 0.8 started.",
            Toast.LENGTH_SHORT
        ).show()

        refreshUi()
    }

    private fun markEvent() {
        if (!prefs.getBoolean("running", false)) {
            Toast.makeText(
                this,
                "Start Drive first.",
                Toast.LENGTH_SHORT
            ).show()
            return
        }

        startService(
            Intent(
                this,
                LocationService::class.java
            ).setAction(
                LocationService.ACTION_MARK_EVENT
            )
        )

        Toast.makeText(
            this,
            "Road event marked.",
            Toast.LENGTH_SHORT
        ).show()
    }

    private fun stopDrive() {
        if (
            !prefs.getBoolean("running", false) &&
            !prefs.getBoolean("trip_active", false)
        ) {
            Toast.makeText(
                this,
                "No active trip.",
                Toast.LENGTH_SHORT
            ).show()
            return
        }

        startService(
            Intent(
                this,
                LocationService::class.java
            ).setAction(
                LocationService.ACTION_STOP_TRIP
            )
        )

        Toast.makeText(
            this,
            "Finalizing persistent trip log…",
            Toast.LENGTH_SHORT
        ).show()

        handler.postDelayed(
            { refreshUi() },
            1200L
        )
    }

    private fun refreshUi() {
        val running =
            prefs.getBoolean("running", false)

        val tripActive =
            prefs.getBoolean("trip_active", false)

        val speed =
            prefs.getFloat("speed_kmh", 0f)

        val lat =
            prefs.getString("lat", null)

        val lon =
            prefs.getString("lon", null)

        val accuracy =
            prefs.getFloat("accuracy_m", -1f)

        val provider =
            prefs.getString("provider", "--") ?: "--"

        val gpsAge =
            prefs.getLong("gps_age_ms", -1L)

        val gpsStale =
            prefs.getBoolean("gps_stale", true)

        val activeLimit =
            prefs.getInt(
                "active_limit_kmh",
                prefs.getInt("threshold_kmh", 80)
            )

        val cameraId =
            prefs.getString("camera_id", "") ?: ""

        val cameraDistance =
            prefs.getFloat("camera_distance_m", -1f)

        val cameraLimit =
            prefs.getInt("camera_limit_kmh", -1)

        val cameraRoad =
            prefs.getString("camera_road_name", "") ?: ""

        val cameraType =
            prefs.getString("camera_type", "") ?: ""

        val roadCross =
            prefs.getFloat("camera_road_cross_m", -1f)

        val forwardDelta =
            prefs.getFloat("camera_forward_delta", -1f)

        val alertLevel =
            prefs.getInt("camera_alert_level", 0)

        val warningDistance =
            prefs.getFloat(
                "camera_warning_distance_m",
                -1f
            )

        val ttc =
            prefs.getFloat("camera_ttc_s", -1f)

        val confirmedCount =
            prefs.getInt(
                "confirmed_camera_count",
                0
            )

        val accel =
            prefs.getBoolean("accel_available", false)

        val gyro =
            prefs.getBoolean("gyro_available", false)

        val linearAccel =
            prefs.getBoolean(
                "linear_accel_available",
                false
            )

        val rotationVector =
            prefs.getBoolean(
                "rotation_vector_available",
                false
            )

        val fusionActive =
            prefs.getBoolean(
                "fusion_active",
                false
            )

        val imuBridge =
            prefs.getBoolean(
                "imu_bridge",
                false
            )

        val gpsFiltered =
            prefs.getFloat(
                "gps_filtered_kmh",
                -1f
            )

        val forwardAccel =
            prefs.getFloat(
                "forward_accel_mps2",
                0f
            )

        val cameraCount =
            prefs.getInt(
                "camera_count",
                0
            )

        val liveLog =
            prefs.getString(
                "active_log_name",
                null
            ) ?: prefs.getString(
                "last_log_name",
                null
            )

        speedText.text =
            String.format(
                Locale.US,
                "%.0f km/h",
                speed
            )

        speedText.setTextColor(
            if (
                running &&
                !gpsStale &&
                speed >= activeLimit + 1f
            ) {
                Color.rgb(210, 35, 35)
            } else {
                Color.rgb(20, 91, 210)
            }
        )

        val dynamic =
            cameraId.isNotEmpty() &&
            cameraLimit > 0

        limitText.text =
            if (dynamic) {
                "LIMIT $activeLimit km/h • CAMERA"
            } else {
                "LIMIT $activeLimit km/h"
            }

        cameraText.text =
            if (
                cameraId.isNotEmpty() &&
                cameraDistance >= 0f
            ) {
                val road =
                    if (cameraRoad.isNotBlank()) {
                        " • $cameraRoad"
                    } else ""

                val lim =
                    if (cameraLimit > 0) {
                        " • $cameraLimit km/h"
                    } else ""

                val adaptive =
                    if (warningDistance > 0f) {
                        " • warn ${warningDistance.toInt()}m" +
                            " • L$alertLevel" +
                            if (ttc > 0f) {
                                " • ${ttc.toInt()}s"
                            } else ""
                    } else ""

                val diag =
                    " • cross ${roadCross.toInt()}m" +
                        " • fwdΔ ${forwardDelta.toInt()}°"

                "CONFIRMED " +
                    if (cameraType == "red_light") {
                        "red-light"
                    } else {
                        "speed"
                    } +
                    " camera $cameraId" +
                    " • ${cameraDistance.toInt()}m" +
                    lim +
                    road +
                    adaptive +
                    diag
            } else {
                "Confirmed camera: --"
            }

        gpsText.text =
            when {
                !running ->
                    "GPS: stopped"

                gpsAge < 0 ->
                    "GPS: waiting for valid speed fix"

                gpsStale ->
                    String.format(
                        Locale.US,
                        "GPS: STALE • %.1f s • acc %.1f m",
                        gpsAge / 1000.0,
                        accuracy
                    )

                imuBridge ->
                    String.format(
                        Locale.US,
                        "GPS: IMU BRIDGE • %.1f s • GPS %.0f km/h",
                        gpsAge / 1000.0,
                        gpsFiltered
                    )

                fusionActive ->
                    String.format(
                        Locale.US,
                        "GPS+IMU: FUSED • GPS %.0f km/h • a %.2f m/s²",
                        gpsFiltered,
                        forwardAccel
                    )

                else ->
                    String.format(
                        Locale.US,
                        "GPS: OK • age %.1f s • acc %.1f m",
                        gpsAge / 1000.0,
                        accuracy
                    )
            }

        gpsText.setTextColor(
            if (gpsStale && running) {
                Color.rgb(200, 60, 30)
            } else {
                Color.rgb(35, 120, 60)
            }
        )

        statusText.text =
            when {
                running ->
                    "ACTIVE • $provider • $cameraCount DB cameras • " +
                        "$confirmedCount confirmed • LIVE LOG"

                tripActive ->
                    "Trip interrupted • persistent log waiting for recovery"

                else ->
                    "Stopped"
            }

        coordsText.text =
            if (lat != null && lon != null) {
                "Location: $lat, $lon"
            } else {
                "Location: --"
            }

        sensorText.text =
            "Sensors: Accel ${if (accel) "OK" else "--"}" +
                " • Gyro ${if (gyro) "OK" else "--"}" +
                " • Linear ${if (linearAccel) "OK" else "--"}" +
                " • Rotation ${if (rotationVector) "OK" else "--"}"

        logText.text =
            if (liveLog != null) {
                "Trip log: Downloads/MOJODrive/$liveLog"
            } else {
                "Trip log: --"
            }
    }
}
