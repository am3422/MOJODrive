# MOJO Drive regression tools

`baseline_guard.py` locks the field-validated v0.12 runtime engine in CI. It fails the build if a protected camera/location/speed function or critical threshold changes accidentally.

`replay_harness.py` replays the locked camera matcher against four successful field traces (three v0.12 trips plus the successful v0.13 shadow trip). `--self-test` must reproduce all **57/57** first-warning encounters in the exact recorded order.

`build_road_geometry_db.py` rebuilds the diagnostic road-geometry SQLite DB from proven near-pass field traces. It can retain multiple distinct, field-proven approaches for one camera. In v0.14 this DB remains **shadow only** and never changes alerts.

`road_geometry_shadow_test.py` validates the conservative multi-approach shadow policy against field-derived regression cases, including the v0.13 `14302` false-conflict discovery and both the abandoned and real `14427` approaches.

Useful commands:

```bash
python3 tools/baseline_guard.py
python3 tools/replay_harness.py --self-test
python3 tools/road_geometry_shadow_test.py
```
