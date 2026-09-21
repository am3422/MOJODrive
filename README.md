# MOJO Drive — MVP 0.2

Feasibility build for real driving tests.

## Added in 0.2
- GPS current speed / bearing / accuracy
- Foreground tracking with screen locked
- Overspeed alert
- Accelerometer logging (~20 Hz)
- Gyroscope logging (~20 Hz)
- Sensor records timestamp-matched with the latest GPS location/speed
- Automatic CSV export when STOP & SAVE LOG is pressed
- Log destination on Android 10+: `Downloads/MOJODrive/`

## CSV record types
- `START`
- `GPS`
- `ACCEL`
- `GYRO`
- `OVERSPEED_ALERT`
- `STOP`

Use the CSV after a real trip to analyze motion start/stop, braking/acceleration, bumps/shocks, GPS quality, and candidate speed bumps.

For safety, never interact with the phone while driving.
