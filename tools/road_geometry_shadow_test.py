#!/usr/bin/env python3
"""Regression test for the v0.14 conservative road-geometry shadow policy.

The production Android evaluator remains SHADOW ONLY. This test uses the same policy and
bundled road_geometry.sqlite to ensure field-proven examples do not regress before an APK
is built. In particular, the real 14302 pass must not be called a conflict, while the
abandoned 14427 branch may become a conflict only after the car is close to learned route
geometry and strongly heading away from it.
"""
from __future__ import annotations
import json, math, sqlite3, sys
from pathlib import Path

SUPPORT_DISTANCE_M = 70.0
WEAK_SUPPORT_DISTANCE_M = 160.0
COVERAGE_DISTANCE_M = 180.0
SUPPORT_HEADING_DEG = 50.0
WEAK_SUPPORT_HEADING_DEG = 65.0
CONFLICT_HEADING_DEG = 105.0
POLICY = "conservative_multi_approach_v2"

ROOT = Path(__file__).resolve().parents[1]
DB = ROOT / "app/src/main/assets/road_geometry.sqlite"
CASES = ROOT / "tools/fixtures/v013_shadow_cases.json"


def angle_diff(a: float, b: float) -> float:
    d = abs(a - b) % 360.0
    return 360.0 - d if d > 180.0 else d


def normalize_bearing(v: float) -> float:
    return v % 360.0


def nearest_to_polyline(lat: float, lon: float, points: list[tuple[float, float]]) -> tuple[float, float] | None:
    if len(points) < 2:
        return None
    meters_per_lon = 111_320.0 * math.cos(math.radians(lat))
    meters_per_lat = 110_540.0
    best_distance = float("inf")
    best_bearing = 0.0
    for a, b in zip(points, points[1:]):
        ax = (a[1] - lon) * meters_per_lon
        ay = (a[0] - lat) * meters_per_lat
        bx = (b[1] - lon) * meters_per_lon
        by = (b[0] - lat) * meters_per_lat
        vx, vy = bx - ax, by - ay
        vv = vx * vx + vy * vy
        if vv < 1e-6:
            continue
        t = max(0.0, min(1.0, -(ax * vx + ay * vy) / vv))
        qx, qy = ax + t * vx, ay + t * vy
        dist = math.hypot(qx, qy)
        if dist < best_distance:
            best_distance = dist
            best_bearing = normalize_bearing(math.degrees(math.atan2(vx, vy)))
    if not math.isfinite(best_distance):
        return None
    return best_distance, best_bearing


def classify(distance_m: float, heading_delta_deg: float, bearing_available: bool) -> str:
    if distance_m > COVERAGE_DISTANCE_M:
        return "outside_coverage"
    if not bearing_available:
        return "position_only"
    if distance_m <= SUPPORT_DISTANCE_M and heading_delta_deg <= SUPPORT_HEADING_DEG:
        return "support"
    if distance_m <= WEAK_SUPPORT_DISTANCE_M and heading_delta_deg <= WEAK_SUPPORT_HEADING_DEG:
        return "weak_support"
    if heading_delta_deg >= CONFLICT_HEADING_DEG:
        return "conflict"
    return "uncertain"


def load_segments() -> tuple[dict[tuple[str, str], list[dict]], dict[str, str]]:
    con = sqlite3.connect(DB)
    try:
        meta = dict(con.execute("SELECT key,value FROM meta"))
        segments: dict[tuple[str, str], list[dict]] = {}
        for sid, cid, typ, source, confidence in con.execute(
            "SELECT segment_id,camera_id,camera_type,source_kind,confidence FROM road_segments ORDER BY segment_id"
        ):
            pts = [
                (lat, lon)
                for _, lat, lon in con.execute(
                    "SELECT seq,latitude,longitude FROM road_points WHERE segment_id=? ORDER BY seq", (sid,)
                )
            ]
            segments.setdefault((str(cid), typ), []).append(
                {"id": sid, "source": source, "confidence": confidence, "points": pts}
            )
        integrity = con.execute("PRAGMA integrity_check").fetchone()[0]
        if integrity != "ok":
            raise RuntimeError(f"road geometry DB integrity_check={integrity}")
        return segments, meta
    finally:
        con.close()


def evaluate(segments: dict, camera_id: str, camera_type: str, lat: float, lon: float, bearing: float | None):
    candidates = []
    for seg in segments.get((str(camera_id), camera_type), []):
        nearest = nearest_to_polyline(lat, lon, seg["points"])
        if nearest is None:
            continue
        distance, route_bearing = nearest
        delta = angle_diff(bearing, route_bearing) if bearing is not None else 180.0
        verdict = classify(distance, delta, bearing is not None)
        candidates.append({**seg, "distance": distance, "delta": delta, "verdict": verdict})
    if not candidates:
        return None, []

    def choose(verdict: str):
        xs = [c for c in candidates if c["verdict"] == verdict]
        if not xs:
            return None
        return min(xs, key=lambda c: c["distance"] + min(c["delta"], 180.0) * 0.35)

    chosen = (
        choose("support")
        or choose("weak_support")
        or choose("uncertain")
        or choose("position_only")
        or choose("conflict")
        or min(candidates, key=lambda c: c["distance"])
    )
    return chosen, candidates


def main() -> int:
    segments, meta = load_segments()
    failures = []

    if meta.get("engine_effect") != "false":
        failures.append("DB meta engine_effect must remain false")
    if meta.get("mode") != "shadow_only":
        failures.append("DB meta mode must remain shadow_only")
    if meta.get("policy") != POLICY:
        failures.append(f"DB policy {meta.get('policy')} != {POLICY}")

    data = json.loads(CASES.read_text(encoding="utf-8"))
    print(f"ROAD GEOMETRY SHADOW TEST: policy={POLICY} cases={len(data['cases'])}")
    for case in data["cases"]:
        chosen, all_candidates = evaluate(
            segments,
            case["camera_id"],
            case["camera_type"],
            float(case["lat"]),
            float(case["lon"]),
            None if case.get("bearing") is None else float(case["bearing"]),
        )
        if chosen is None:
            failures.append(f"{case['name']}: no geometry")
            print(f"  FAIL {case['name']}: no geometry")
            continue
        verdict = chosen["verdict"]
        allowed = case["allowed"]
        status = "PASS" if verdict in allowed else "FAIL"
        print(
            f"  {status} {case['name']}: {verdict} "
            f"d={chosen['distance']:.1f}m headingDelta={chosen['delta']:.1f}° "
            f"segment={chosen['id']} candidates={len(all_candidates)}"
        )
        if verdict not in allowed:
            failures.append(f"{case['name']}: verdict={verdict}, allowed={allowed}")

    if failures:
        print("ROAD GEOMETRY SHADOW TEST: FAIL")
        for f in failures:
            print(" -", f)
        return 2

    print("ROAD GEOMETRY SHADOW TEST: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
