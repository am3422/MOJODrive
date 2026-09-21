# MOJO Drive — MVP 0.1

First feasibility build for the MOJOTech driving-assistant idea.

## What this build tests
- Phone GPS without any external hardware
- Current speed from Android location
- Foreground location tracking while the app is not visible / screen is locked
- User-set overspeed threshold
- One-shot sound + vibration when crossing the threshold
- Automatic re-arm after speed falls 5 km/h below the threshold
- Local CSV trip logging

## Not included yet
- Camera database
- Speed-bump database
- Heading filtering
- Map matching
- Phone accelerometer/gyro
- BLE / ESP32
- Cloud/server/login

## Build in GitHub Actions
Push this project to a GitHub repository. The included workflow builds:
`app/build/outputs/apk/debug/app-debug.apk`

Open **Actions > Build MOJO Drive APK**, choose **Run workflow**, then download the artifact named:
`MOJODrive-MVP-0.1-debug`

## Phone test
1. Install the APK.
2. Allow precise location.
3. Allow notifications if Android asks.
4. Make sure phone Location/GPS is ON.
5. Set a safe test threshold.
6. Tap START DRIVE while the app is visible.
7. Confirm the persistent `MOJO Drive active` notification appears.
8. Lock the phone and drive normally.
9. Confirm speed continues to update in the notification/app after unlocking.
10. For overspeed testing, choose a threshold appropriate for a legal/safe road and obey all traffic laws.

Do not interact with the phone while driving. Have a passenger observe/debug if needed.
