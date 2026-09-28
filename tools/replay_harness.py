#!/usr/bin/env python3
"""MOJO Drive camera-engine replay harness.

Replays the field-validated v0.12 camera matcher from recorded LOCATION rows using the
same bundled camera DB. It intentionally excludes sound/vibration scheduling and IMU
integration; recorded fused speed, bearing, accuracy and location-gap data are used as
inputs. On the three golden v0.12 trips it reproduces the exact first-warning sequence.

Examples:
  python tools/replay_harness.py --self-test
  python tools/replay_harness.py path/to/MOJODrive_Trip.csv
  python tools/replay_harness.py --json report.json path/to/trip1.csv path/to/trip2.csv
"""
from __future__ import annotations
import argparse, csv, hashlib, json, math, os, re, sqlite3, sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

CAMERA_SEARCH_M = 2300.0
CAMERA_CONFIRM_EXTRA_M = 500.0
CAMERA_CLUSTER_BASE_M = 40.0
CAMERA_CLUSTER_EXTENDED_M = 110.0
CAMERA_CLUSTER_AXIS_DELTA_DEG = 18.0
CAMERA_CLUSTER_DIRECTED_DELTA_DEG = 22.0
PASS_BEHIND_DEG = 100.0
PASS_DISTANCE_GROWTH_M = 25.0
NEAR_PASS_AUDIT_M = 140.0
LOCATION_STALE_MS = 8000
EARTH_R = 6371000.0

@dataclass
class Camera:
    id: str
    type: str
    lat: float
    lon: float
    speed_limit: Optional[int]
    road_name: str
    road_highway: str
    road_distance: Optional[float]
    speed_source: str
    speed_conf: str
    rb: Optional[float]
    oneway: bool
    direction_mode: str
    direction_conf: str

@dataclass
class Cluster:
    key: str
    camera: Camera
    ids: list[str]

@dataclass
class State:
    phase: str = "TRACKING"
    confirm_hits: int = 0
    approach_hits: int = 0
    away_hits: int = 0
    hard_bad_hits: int = 0
    warned: bool = False
    last_seen: int = 0
    transition: int = 0
    last_distance: float = 1e30
    min_distance: float = 1e30
    near_audited: bool = False


def normalize_bearing(v: float) -> float:
    return v % 360.0


def angle_difference(a: float, b: float) -> float:
    d = abs(a - b) % 360.0
    return 360.0 - d if d > 180.0 else d


def distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = p2 - p1
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * EARTH_R * math.atan2(math.sqrt(a), math.sqrt(max(0.0, 1.0 - a)))


def bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return normalize_bearing(math.degrees(math.atan2(y, x)))


def has_reliable_directed_bearing(c: Camera) -> bool:
    return c.oneway and c.rb is not None and c.direction_conf.lower() in ("high", "medium")


def normalize_road_name(v: str) -> str:
    return " ".join((v or "").strip().lower().split())


def valid_dynamic_limit(c: Camera) -> Optional[int]:
    if c.speed_limit is None or not 20 <= c.speed_limit <= 150:
        return None
    if c.speed_conf.lower() not in ("high", "medium"):
        return None
    if (c.road_distance if c.road_distance is not None else 999.0) > 30.0:
        return None
    return c.speed_limit


def camera_metadata_score(c: Camera) -> int:
    score = 0
    if valid_dynamic_limit(c) is not None:
        score += 5
    if c.rb is not None:
        score += 3
    if c.direction_conf.lower() == "high":
        score += 3
    elif c.direction_conf.lower() == "medium":
        score += 2
    if (c.road_distance if c.road_distance is not None else 999.0) <= 30.0:
        score += 2
    if c.road_name.strip():
        score += 1
    return score


def bearings_cluster_compatible(a: Camera, b: Camera) -> bool:
    if a.rb is None or b.rb is None:
        return True
    if has_reliable_directed_bearing(a) and has_reliable_directed_bearing(b):
        return angle_difference(a.rb, b.rb) <= CAMERA_CLUSTER_DIRECTED_DELTA_DEG
    return min(
        angle_difference(a.rb, b.rb),
        angle_difference(a.rb, normalize_bearing(b.rb + 180.0)),
    ) <= CAMERA_CLUSTER_AXIS_DELTA_DEG


def cameras_cluster_compatible(a: Camera, b: Camera) -> bool:
    if a.type != b.type:
        return False
    d = distance_m(a.lat, a.lon, b.lat, b.lon)
    if d > CAMERA_CLUSTER_EXTENDED_M:
        return False
    if d <= CAMERA_CLUSTER_BASE_M:
        if a.rb is None or b.rb is None:
            return True
        return bearings_cluster_compatible(a, b)
    if a.rb is None or b.rb is None or not bearings_cluster_compatible(a, b):
        return False
    if a.speed_limit is not None and b.speed_limit is not None and abs(a.speed_limit - b.speed_limit) > 5:
        return False
    ar, br = normalize_road_name(a.road_name), normalize_road_name(b.road_name)
    if ar and br and ar != br:
        return False
    return True


def load_clusters(db_path: str) -> list[Cluster]:
    con = sqlite3.connect(db_path)
    rows = con.execute(
        "SELECT camera_id,camera_type,latitude,longitude,speed_limit,road_name,road_highway,"
        "road_distance_m,speed_source,speed_confidence,road_bearing,oneway,direction_mode,"
        "direction_confidence FROM cameras"
    ).fetchall()
    con.close()
    remaining = [
        Camera(str(r[0]), r[1], r[2], r[3], r[4], r[5] or "", r[6] or "", r[7],
               r[8] or "", r[9] or "unknown", r[10], bool(r[11]), r[12] or "unknown", r[13] or "unknown")
        for r in rows
    ]
    out: list[Cluster] = []
    while remaining:
        seed = remaining.pop(0)
        members = [seed]
        changed = True
        while changed:
            changed = False
            kept = []
            for cand in remaining:
                if any(cameras_cluster_compatible(m, cand) for m in members):
                    members.append(cand)
                    changed = True
                else:
                    kept.append(cand)
            remaining = kept
        rep = max(members, key=camera_metadata_score)
        avg_lat = sum(m.lat for m in members) / len(members)
        avg_lon = sum(m.lon for m in members) / len(members)
        synthetic = Camera(rep.id, rep.type, avg_lat, avg_lon, rep.speed_limit, rep.road_name,
                           rep.road_highway, rep.road_distance, rep.speed_source, rep.speed_conf,
                           rep.rb, rep.oneway, rep.direction_mode, rep.direction_conf)
        ids = sorted(set(m.id for m in members))
        out.append(Cluster(rep.type + ":" + "+".join(ids), synthetic, ids))
    return out


def adaptive_warning_distance(speed_kmh: float, limit: Optional[int], camera_type: str) -> float:
    v = max(5.0, speed_kmh) / 3.6
    base_lead = max(24.0, min(32.0, 24.0 + max(0.0, speed_kmh - 45.0) * 0.12))
    warning = v * base_lead
    if limit is not None and speed_kmh > limit:
        target = limit / 3.6
        decel = max(0.0, (v * v - target * target) / (2.0 * 1.6))
        warning = max(warning, decel + v * 5.0 + 90.0)
    if camera_type == "red_light":
        warning = max(warning, v * 26.0)
    return max(440.0, min(1250.0, warning))


def build_geometry(cl: Cluster, st: State, now: int, lat: float, lon: float,
                   vehicle_bearing: float, speed: float, accuracy: float) -> dict:
    cam = cl.camera
    d = distance_m(lat, lon, cam.lat, cam.lon)
    b_to = bearing_deg(lat, lon, cam.lat, cam.lon)
    f_delta = angle_difference(vehicle_bearing, b_to)
    vehicle_forward = d * math.cos(math.radians(f_delta))
    vehicle_cross = abs(d * math.sin(math.radians(f_delta)))
    rb = cam.rb
    if rb is not None:
        axis_delta = min(angle_difference(vehicle_bearing, rb),
                         angle_difference(vehicle_bearing, normalize_bearing(rb + 180.0)))
        direction_delta = angle_difference(vehicle_bearing, rb) if has_reliable_directed_bearing(cam) else axis_delta
        camera_axis_delta = min(angle_difference(rb, b_to),
                                angle_difference(normalize_bearing(rb + 180.0), b_to))
        road_cross = abs(d * math.sin(math.radians(camera_axis_delta)))
    else:
        axis_delta = direction_delta = 0.0
        road_cross = vehicle_cross
    curve_cross = min(vehicle_cross, road_cross) if rb is not None else vehicle_cross

    if d > 1500: base_cross = 230.0
    elif d > 1100: base_cross = 210.0
    elif d > 800: base_cross = 195.0
    elif d > 600: base_cross = 178.0
    elif d > 450: base_cross = 145.0
    elif d > 300: base_cross = 115.0
    else: base_cross = 80.0
    allowance = max(0.0, min(20.0, accuracy * 1.2)) if accuracy >= 0 else 10.0
    hard_cross_limit = base_cross + allowance
    hard_cross_ok = curve_cross <= hard_cross_limit

    if d > 1100: forward_limit, direction_limit = 45.0, 42.0
    elif d > 750: forward_limit, direction_limit = 50.0, 48.0
    elif d > 550: forward_limit, direction_limit = 55.0, 55.0
    elif d > 350: forward_limit, direction_limit = 68.0, 65.0
    else: forward_limit, direction_limit = 85.0, 78.0
    direction_gate_ok = f_delta <= forward_limit and (rb is None or direction_delta <= direction_limit)
    hard_ok = vehicle_forward > 8.0 and hard_cross_ok and direction_gate_ok

    limit = valid_dynamic_limit(cam)
    warning = adaptive_warning_distance(speed, limit, cam.type)
    speed_mps = max(1.5, speed / 3.6)
    ttc = vehicle_forward / speed_mps if vehicle_forward > 0 else -1.0
    dt = (now - st.last_seen) / 1000.0 if st.last_seen > 0 else 0.0
    approach = st.last_distance - d if st.last_distance < 1e29 else 0.0
    approach_rate = approach / dt if dt > 0 else 0.0

    score = 0
    if hard_cross_ok:
        if curve_cross <= hard_cross_limit * 0.35: score += 3
        elif curve_cross <= hard_cross_limit * 0.65: score += 2
        else: score += 1
    if f_delta <= 20: score += 3
    elif f_delta <= 35: score += 2
    elif f_delta <= 55: score += 1
    if rb is not None:
        if direction_delta <= 20: score += 2
        elif direction_delta <= 40: score += 1
    else:
        score += 1
    if vehicle_forward > 8: score += 1
    if approach > 2: score += 2
    if 0 <= accuracy <= 8: score += 2
    elif 0 <= accuracy <= 18: score += 1

    if d > 1200: required_score = 9
    elif d > 850: required_score = 8
    elif d > 550: required_score = 7
    elif d > 320: required_score = 6
    else: required_score = 5
    if d > 850: required_hits = 3
    elif d > 600: required_hits = 3
    elif d > 350: required_hits = 2
    else: required_hits = 1

    return dict(d=d, vehicle_forward=vehicle_forward, f_delta=f_delta, hard_ok=hard_ok,
                score=score, required_score=required_score, required_hits=required_hits,
                warning=warning, ttc=ttc, approach=approach, approach_rate=approach_rate)


def replay(db_path: str, log_path: str) -> dict:
    clusters = load_clusters(db_path)
    states: dict[str, State] = {}
    warnings = []
    near_passes = []
    location_count = 0
    max_gap_ms = 0
    previous_t = None

    with open(log_path, encoding="utf-8") as f:
        for row in csv.DictReader(f):
            if row.get("record_type") not in ("LOCATION", "LOCATION_ONLY"):
                continue
            if not row.get("lat") or not row.get("lon"):
                continue
            location_count += 1
            now = int(row["wall_time_ms"])
            if previous_t is not None:
                max_gap_ms = max(max_gap_ms, now - previous_t)
            previous_t = now
            if row.get("bearing_valid", "").lower() != "true" or not row.get("bearing_deg"):
                continue
            lat, lon = float(row["lat"]), float(row["lon"])
            vehicle_bearing = float(row["bearing_deg"])
            speed = float(row.get("fused_speed_kmh") or 0.0)
            accuracy = float(row.get("gps_accuracy_m") or -1.0)
            m = re.search(r"locationGapMs=(\d+)", row.get("details") or "")
            recovery_gap = int(m.group(1)) if m else 0

            for cl in clusters:
                direct = distance_m(lat, lon, cl.camera.lat, cl.camera.lon)
                if direct > CAMERA_SEARCH_M:
                    continue
                st = states.setdefault(cl.key, State())
                if st.phase in ("PASSED", "ABANDONED") and direct > 1200 and now - st.transition > 30000:
                    st = State()
                    states[cl.key] = st
                if st.phase in ("PASSED", "ABANDONED"):
                    st.last_seen = now
                    continue

                g = build_geometry(cl, st, now, lat, lon, vehicle_bearing, speed, accuracy)
                st.last_seen = now
                st.min_distance = min(st.min_distance, g["d"])
                if st.last_distance < 1e29:
                    if g["approach"] > 2:
                        st.approach_hits = min(10, st.approach_hits + 1)
                        st.away_hits = max(0, st.away_hits - 1)
                    elif g["approach"] < -3:
                        st.away_hits = min(10, st.away_hits + 1)
                        st.approach_hits = max(0, st.approach_hits - 1)

                within = g["d"] <= max(950.0, g["warning"] + CAMERA_CONFIRM_EXTRA_M)
                strong = st.approach_hits >= 2
                fast = g["d"] <= 500 and strong and g["hard_ok"] and g["score"] >= g["required_score"] - 1
                eligible = within and g["hard_ok"] and (g["score"] >= g["required_score"] or fast) and (
                    st.approach_hits >= 1 or g["d"] <= 250
                )
                if st.phase == "TRACKING":
                    if eligible:
                        st.confirm_hits += 1
                        needed = 1 if fast else g["required_hits"]
                        if st.confirm_hits >= needed:
                            st.phase = "CONFIRMED"
                            st.transition = now
                            st.hard_bad_hits = 0
                    else:
                        st.confirm_hits = max(0, st.confirm_hits - 1)

                if g["d"] <= NEAR_PASS_AUDIT_M and not st.near_audited:
                    st.near_audited = True
                    near_passes.append({"cluster": cl.key, "time_ms": now, "distance_m": round(g["d"], 2),
                                        "phase": st.phase, "warned": st.warned})

                if st.phase == "CONFIRMED":
                    st.hard_bad_hits = 0 if g["hard_ok"] else st.hard_bad_hits + 1
                    if recovery_gap > LOCATION_STALE_MS and g["vehicle_forward"] <= 0:
                        st.phase = "PASSED"; st.transition = now; st.last_distance = g["d"]; continue
                    clearly_behind = g["f_delta"] >= PASS_BEHIND_DEG
                    growing = st.last_distance < 1e29 and g["d"] > st.min_distance + PASS_DISTANCE_GROWTH_M
                    if clearly_behind and growing:
                        st.away_hits += 1
                        if st.away_hits >= 2:
                            st.phase = "PASSED"; st.transition = now; st.last_distance = g["d"]; continue
                    if st.hard_bad_hits >= 4 and st.away_hits >= 3 and g["d"] > 250:
                        st.phase = "ABANDONED"; st.transition = now; st.last_distance = g["d"]; continue
                    in_warning_zone = g["vehicle_forward"] <= g["warning"] + 90.0
                    if g["vehicle_forward"] > 0 and g["f_delta"] < PASS_BEHIND_DEG and g["hard_ok"] and in_warning_zone:
                        if not st.warned:
                            st.warned = True
                            warnings.append({
                                "cluster": cl.key,
                                "time_ms": now,
                                "distance_m": round(g["d"], 2),
                                "forward_m": round(g["vehicle_forward"], 2),
                                "ttc_s": round(g["ttc"], 2),
                            })
                st.last_distance = g["d"]

    return {"log": os.path.basename(log_path), "locations": location_count, "max_location_gap_ms": max_gap_ms,
            "warning_count": len(warnings), "warnings": warnings, "near_passes": near_passes}


def self_test(root: Path, db: Path, golden_path: Path) -> int:
    golden = json.loads(golden_path.read_text(encoding="utf-8"))
    digest = hashlib.sha256(db.read_bytes()).hexdigest()
    failures = []
    if digest != golden["camera_db_sha256"]:
        failures.append(f"camera DB SHA mismatch: {digest}")
    for fixture_name, expected in golden["trips"].items():
        result = replay(str(db), str(root / "tools" / "fixtures" / fixture_name))
        actual = [w["cluster"] for w in result["warnings"]]
        if actual != expected:
            failures.append(f"{fixture_name}: warning sequence differs\n  expected={expected}\n  actual={actual}")
        else:
            print(f"REPLAY PASS {fixture_name}: {len(actual)} first warnings, exact sequence match")
    if failures:
        print("REPLAY SELF-TEST: FAIL")
        for f in failures:
            print(" -", f)
        return 3
    total = sum(len(v) for v in golden["trips"].values())
    print(f"REPLAY SELF-TEST: PASS — {total}/{total} golden first-warning encounters reproduced exactly")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("logs", nargs="*")
    ap.add_argument("--camera-db", default="app/src/main/assets/shiraz_cameras.sqlite")
    ap.add_argument("--golden", default="tools/golden_v012_replay.json")
    ap.add_argument("--self-test", action="store_true")
    ap.add_argument("--json", dest="json_out")
    args = ap.parse_args()
    root = Path(__file__).resolve().parents[1]
    db = Path(args.camera_db)
    if not db.is_absolute(): db = root / db
    if args.self_test:
        golden = Path(args.golden)
        if not golden.is_absolute(): golden = root / golden
        return self_test(root, db, golden)
    if not args.logs:
        ap.error("provide one or more trip CSV files, or use --self-test")
    reports = [replay(str(db), p) for p in args.logs]
    for r in reports:
        seq = [w["cluster"] for w in r["warnings"]]
        print(f"{r['log']}: locations={r['locations']} maxGap={r['max_location_gap_ms']/1000:.2f}s firstWarnings={r['warning_count']}")
        print("  " + ", ".join(seq))
    if args.json_out:
        Path(args.json_out).write_text(json.dumps(reports, indent=2), encoding="utf-8")
        print("wrote", args.json_out)
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
