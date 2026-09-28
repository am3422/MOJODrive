#!/usr/bin/env python3
"""CI guard for the intentionally narrow MVP 0.15 live influence.

This is NOT a general road-geometry permission. It verifies that the only live veto is:
- explicit user-confirmed negative-route evidence,
- first alert only,
- live GPS fix only (never GPS bridge),
- and that the bundled DB currently grants that evidence only to camera 14550.
"""
from __future__ import annotations

import re
import sqlite3
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "app/src/main/java/com/mojotech/mojodrive/LocationService.kt"
DB = ROOT / "app/src/main/assets/road_geometry.sqlite"
BEGIN = "// BEGIN V0.15 GUARDED NEGATIVE ROUTE VETO"
END = "// END V0.15 GUARDED NEGATIVE ROUTE VETO"


def main() -> int:
    failures: list[str] = []
    src = SRC.read_text(encoding="utf-8")

    if src.count(BEGIN) != 1 or src.count(END) != 1:
        failures.append("guarded veto block markers must exist exactly once")
        block = ""
    else:
        block = src[src.index(BEGIN):src.index(END) + len(END)]

    required = {
        "first-alert only": "!state.warned",
        "no GPS-bridge veto": "!bridge",
        "explicit negative verdict": 'explicitNegative?.verdict == "negative_route"',
        "explicit evidence kind": 'explicitNegative.evidenceKind == "explicit_negative"',
        "observable suppression log": 'CAMERA_NEGATIVE_ROUTE_SUPPRESSED',
        "suppression returns no alert": "return null",
    }
    for label, token in required.items():
        if token not in block:
            failures.append(f"missing {label}: {token}")

    for verdict in ("conflict", "outside_coverage", "uncertain", "support", "weak_support"):
        if f'verdict == "{verdict}"' in block:
            failures.append(f"generic verdict illegally allowed to veto: {verdict}")

    if src.count('CAMERA_NEGATIVE_ROUTE_SUPPRESSED') != 1:
        failures.append("suppression event must appear exactly once in LocationService")
    if "explicitNegativeSuppressionLogged" not in src:
        failures.append("per-track suppression-log latch missing")
    if "explicit_negative_veto=true" not in src:
        failures.append("startup marker does not disclose explicit negative veto")

    con = sqlite3.connect(DB)
    try:
        if con.execute("PRAGMA integrity_check").fetchone()[0] != "ok":
            failures.append("road_geometry.sqlite integrity_check failed")
        meta = dict(con.execute("SELECT key,value FROM meta"))
        if meta.get("mode") != "guarded_negative_veto":
            failures.append(f"DB mode={meta.get('mode')} expected guarded_negative_veto")
        if meta.get("generic_geometry_engine_effect") != "false":
            failures.append("generic geometry engine effect must stay false")
        if meta.get("explicit_negative_veto") != "true":
            failures.append("explicit negative veto metadata must be true")

        negative_cameras = {
            str(r[0]) for r in con.execute("SELECT DISTINCT camera_id FROM exclusion_segments")
        }
        if negative_cameras != {"14550"}:
            failures.append(
                f"v0.15 explicit-negative DB scope changed: {sorted(negative_cameras)} != ['14550']"
            )
        labels = {
            str(r[0]) for r in con.execute("SELECT DISTINCT label FROM exclusion_segments")
        }
        expected_labels = {
            "14550_opposite_level_approach",
            "14550_hill_return_false_warning",
        }
        if labels != expected_labels:
            failures.append(f"negative-route labels changed: {sorted(labels)}")
    finally:
        con.close()

    if failures:
        print("NEGATIVE ROUTE VETO GUARD: FAIL")
        for f in failures:
            print(" -", f)
        return 2

    print("NEGATIVE ROUTE VETO GUARD: PASS")
    print(" - generic road geometry remains shadow-only")
    print(" - live veto requires explicit_negative + negative_route")
    print(" - veto is first-alert only and disabled during GPS bridge")
    print(" - current explicit-negative scope: camera 14550 only (2 field-confirmed route segments)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
