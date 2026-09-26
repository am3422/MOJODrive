# MOJO Drive regression tools

`baseline_guard.py` locks the field-validated v0.12 runtime engine in CI.

`replay_harness.py` replays the camera matcher against the three successful v0.12 field traces. `--self-test` must reproduce all 38 first-warning encounters in the exact order.

`build_road_geometry_db.py` builds the diagnostic road-geometry SQLite DB from proven near-pass field traces. v0.13 uses that DB in **shadow mode only**; it never changes alerts.
