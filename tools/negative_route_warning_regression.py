#!/usr/bin/env python3
"""Field-warning regression for MVP 0.15 explicit-negative veto.

The unchanged v0.12 core emitted 20 first warnings in the 2026-09-28 v0.14 trip.
User ground truth says only the final 14550 warning was false (opposite carriageway/level
while the vehicle was on the hill route). This test verifies the new explicit-negative
geometry veto matches exactly that warning and none of the 19 earlier first warnings.
"""
from __future__ import annotations
import json
from pathlib import Path
from road_geometry_shadow_test import load_db, evaluate

ROOT = Path(__file__).resolve().parents[1]
CASES = ROOT / "tools/fixtures/v014_first_warning_cases.json"


def main() -> int:
    _, pos, neg = load_db()
    data = json.loads(CASES.read_text(encoding="utf-8"))
    failures = []
    vetoed = []

    for case in data["cases"]:
        camera_ids = str(case["camera_id"]).split("|")
        matched = False
        details = []
        for cid in camera_ids:
            result = evaluate(
                pos, neg, cid, case["camera_type"],
                float(case["lat"]), float(case["lon"]),
                None if case.get("bearing") is None else float(case["bearing"]),
            )
            if result is None:
                details.append(f"{cid}:none")
                continue
            details.append(
                f"{cid}:{result['verdict']}@{result['distance']:.1f}m/{result['delta']:.1f}deg"
            )
            if result["verdict"] == "negative_route" and result["kind"] == "negative":
                matched = True

        expected = bool(case["expected_explicit_negative_veto"])
        if matched:
            vetoed.append((case["iso_time"], case["camera_id"]))
        status = "PASS" if matched == expected else "FAIL"
        print(f"  {status} {case['iso_time']} camera={case['camera_id']} expectedVeto={expected} actualVeto={matched} {' '.join(details)}")
        if matched != expected:
            failures.append(
                f"{case['iso_time']} camera {case['camera_id']}: expected veto={expected}, actual={matched}"
            )

    if vetoed != [("2026-09-28T10:28:53.785", "14550")]:
        failures.append(f"exact veto set changed: {vetoed}")

    if failures:
        print("NEGATIVE ROUTE WARNING REGRESSION: FAIL")
        for f in failures:
            print(" -", f)
        return 2

    print("NEGATIVE ROUTE WARNING REGRESSION: PASS — 20 core first warnings checked; exactly 14550 is vetoed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
