#!/usr/bin/env python3
"""Apply the one intentionally narrow runtime change for MOJO Drive MVP 0.15.

Why this script exists:
- The large, field-validated LocationService.kt stays in the user's repo untouched except
  for four exact, reviewable edits.
- The script refuses to patch an unexpected source version.
- Generic road geometry remains shadow-only. Only explicitly user-confirmed negative-route
  evidence can veto the FIRST live alert, and never during a GPS bridge.

Expected parent: MOJO Drive MVP 0.14, commit
228de2611c0e3f230357870287b0023c0ec2ed96
"""
from __future__ import annotations

import hashlib
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent
SRC = ROOT / "app/src/main/java/com/mojotech/mojodrive/LocationService.kt"
EXPECTED_PARENT_BLOB = "0c84d897d49f829a953835afaa4f0e7060aabcff"

BEGIN = "// BEGIN V0.15 GUARDED NEGATIVE ROUTE VETO"
END = "// END V0.15 GUARDED NEGATIVE ROUTE VETO"


def git_blob_sha(data: bytes) -> str:
    return hashlib.sha1(b"blob " + str(len(data)).encode("ascii") + b"\0" + data).hexdigest()


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{label}: expected exactly 1 match, found {count}")
    return text.replace(old, new, 1)


def main() -> int:
    if not SRC.exists():
        print(f"ERROR: missing {SRC}")
        return 2

    raw = SRC.read_bytes()
    text = raw.decode("utf-8")

    # Idempotent success: useful if the command was re-run after a completed patch.
    if BEGIN in text and END in text:
        print("V0.15 LocationService guarded negative-route veto is already applied.")
        return 0

    actual_blob = git_blob_sha(raw)
    if actual_blob != EXPECTED_PARENT_BLOB:
        print("ERROR: LocationService.kt is not the expected field-validated v0.14 parent.")
        print(f" expected git blob: {EXPECTED_PARENT_BLOB}")
        print(f" actual   git blob: {actual_blob}")
        print("Refusing to patch so no newer/local work is overwritten.")
        return 3

    text = replace_once(
        text,
        "// v0.13 diagnostic-only road geometry. It never changes camera decisions.",
        "// v0.15: generic road geometry remains shadow-only; only explicit user-confirmed "
        "negative-route evidence may veto a first live alert.",
        "road geometry policy comment",
    )

    text = replace_once(
        text,
        "        var lastPulseElapsed: Long = 0L,\n        var missedDuringGapLogged: Boolean = false\n",
        "        var lastPulseElapsed: Long = 0L,\n"
        "        var missedDuringGapLogged: Boolean = false,\n"
        "        var explicitNegativeSuppressionLogged: Boolean = false\n",
        "camera track state extension",
    )

    text = replace_once(
        text,
        '''                "ROAD_GEOMETRY_SHADOW_READY",\n                "segments=${roadGeometryShadow?.segmentCount ?: 0};" +\n                    "coveredCameras=${roadGeometryShadow?.coveredCameraCount ?: 0};" +\n                    "mode=shadow_only;engine_effect=false;" +\n                    "policy=${RoadGeometryShadow.POLICY}"\n''',
        '''                "ROAD_GEOMETRY_SHADOW_READY",\n                "positiveSegments=${roadGeometryShadow?.positiveSegmentCount ?: 0};" +\n                    "negativeSegments=${roadGeometryShadow?.negativeSegmentCount ?: 0};" +\n                    "coveredCameras=${roadGeometryShadow?.coveredCameraCount ?: 0};" +\n                    "negativeCameras=${roadGeometryShadow?.negativeCoveredCameraCount ?: 0};" +\n                    "mode=guarded_negative_veto;generic_engine_effect=false;" +\n                    "explicit_negative_veto=true;policy=${RoadGeometryShadow.POLICY}"\n''',
        "road geometry startup marker",
    )

    veto_block = '''
        // BEGIN V0.15 GUARDED NEGATIVE ROUTE VETO
        // Deliberately narrow live influence. Generic support/conflict/outside-coverage
        // remains diagnostic-only. We veto only a FIRST, non-bridged alert when the vehicle
        // is on an explicitly user-confirmed false-warning route and both position + heading
        // match that route. Once an alert has started, geometry cannot mute it.
        if (!bridge && !state.warned) {
            val explicitNegative = try {
                roadGeometryShadow?.evaluate(
                    geometry.cluster.memberIds,
                    geometry.cluster.camera.type,
                    latestLat,
                    latestLon,
                    if (latestBearingValid) latestBearing else null
                )
            } catch (_: Exception) {
                null
            }

            if (
                explicitNegative?.verdict == "negative_route" &&
                explicitNegative.evidenceKind == "explicit_negative"
            ) {
                state.alertLevel = 0
                if (!state.explicitNegativeSuppressionLogged) {
                    state.explicitNegativeSuppressionLogged = true
                    appendCameraEvent(
                        "CAMERA_NEGATIVE_ROUTE_SUPPRESSED",
                        geometry,
                        state,
                        "negativeLabel=${explicitNegative.negativeLabel ?: ""};" +
                            "negativeDistanceM=${String.format(Locale.US, "%.1f", explicitNegative.corridorDistanceM)};" +
                            "negativeHeadingDelta=${String.format(Locale.US, "%.1f", explicitNegative.headingDeltaDeg)};" +
                            "bridge=$bridge;policy=${RoadGeometryShadow.POLICY}"
                    )
                }
                return null
            }
        }
        // END V0.15 GUARDED NEGATIVE ROUTE VETO
'''

    text = replace_once(
        text,
        "\n        val level = computeCameraAlertLevel(\n",
        veto_block + "        val level = computeCameraAlertLevel(\n",
        "guarded negative-route veto insertion",
    )

    SRC.write_text(text, encoding="utf-8", newline="\n")
    print("Applied MVP 0.15 guarded negative-route veto to LocationService.kt")
    print("No camera matcher thresholds, location logic, speed fusion, alert cadence, or overspeed constants were changed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
