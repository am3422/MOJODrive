# MOJO Drive MVP 0.4

Android driving assistant prototype for Shiraz.

## Included offline data
- 252 speed cameras
- 15 red-light cameras
- Neshan basemap road match for camera road name/class
- `routable` road speed reference where available
- one-way carriageway geometry bearing and direction confidence

## Runtime behavior
- GPS-only valid speed fixes with stale-GPS protection
- 3-sample median speed filtering
- real carriageway direction filtering for one-way camera roads
- forward/corridor fallback for two-way or uncertain roads
- dynamic warning distance based on current speed
- separate speed-camera and red-light-camera warnings
- repeated overspeed warning every 7 seconds while over the active limit
- accelerometer/gyroscope trip logging
- manual `MARK / BUMP NOW` marker for future bump detector training
- partial wake lock for locked-screen tracking

`routable` is treated as Neshan's road routing/reference speed, not asserted as the legal posted speed limit.
