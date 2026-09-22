# MOJODrive MVP 0.5

Android driving assistant test build for Shiraz.

## 0.5 additions
- GPS + IMU fused speed using Android linear acceleration + rotation vector and a 1D Kalman-style correction
- IMU dead-reckoning bridge for short GPS gaps up to 8 seconds
- GPS remains the absolute long-term speed reference
- Device orientation compensated before projecting acceleration onto the GPS travel bearing
- Fused/GPS speed, forward acceleration, fusion state and IMU-bridge state added to trip CSV
- Automatic `BUMP_CANDIDATE` markers from accelerometer shocks while moving

## Existing 0.4 features preserved
- Offline Shiraz speed and red-light camera database
- Direction-aware carriageway filtering from road bearing/oneway data
- Dynamic road speed reference near high/medium-confidence speed cameras
- Two-stage camera alerts and repeating overspeed alerts
- GPS quality/stale protection
- Accelerometer + gyroscope logging and manual MARK / BUMP NOW
- Foreground service + partial wake lock

The IMU estimate is intentionally limited to short GPS gaps because inertial speed drifts over time.
