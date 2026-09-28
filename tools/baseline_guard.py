#!/usr/bin/env python3
"""MOJO Drive stable-baseline guard with one explicit v0.15 extension allowance.

The field-validated v0.12 camera/location/speed engine remains locked. v0.15 allows exactly
one marked insertion inside computeAlertRequest: the guarded explicit-negative-route FIRST
alert veto. The guard removes that marked block before hashing, so the underlying v0.12
function must still match byte-for-byte. Every other protected function and critical
constant must remain unchanged.
"""
from __future__ import annotations
import argparse, hashlib, re
from pathlib import Path

EXPECTED = {
  "buildCameraClusters": "9d8afb0d53312e75537c46a47015fd6bff0a03ec65e4eca3bd9b2594ef34c077",
  "camerasClusterCompatible": "133cae73cb18ea1840ea673a3ed514c46672fc31f186499370466394e19d2d71",
  "bearingsClusterCompatible": "f8df9a605fee5fbec2884dccf21b78aa7c7a8eaa0c0cbf2270358cba532d23c8",
  "hasReliableDirectedBearing": "bef1ea7c6ab7d861a6daef8773062d44d5f853dafd0038668885e7a538502c97",
  "cameraMetadataScore": "b300e2a9fa9b6c04b63765eecbfa84019377ad390c576f7a2dbd6a2bf9002d77",
  "startLocationPipeline": "7032d021b5d5c13d4ca9335069c355a572861b93e61d11a5e19d4145dab6fcf4",
  "requestFusedLocationUpdates": "d2b82c5cc78bb8547a25ec6831873d9cc21fe1d288c88564a6a0fad29036c465",
  "startLegacyLocationFallback": "fbbff92da73ffe59fae97153bc64dadd9d5355ff585ae99fd3b9d7ccd9aa030e",
  "monitorLocationPipeline": "09cbd1b09b12f107967785e3e11b70aabeb92624139642fe0d458cbd3f53579c",
  "processLocation": "8d7bb6a6095ab32e60a9674ed38a132197c51b3562757b79ae80df7eae2de24a",
  "updateDerivedSpeed": "e37175934972e9691fb5a091db31fd18a094fabefacba246d24604b59c36c107",
  "prepareForLocationGapRecovery": "7f339ffbc086d6846ee77aa01125264f793407078612fbfd586ed918e28d77a8",
  "predictFusion": "bf9a0e3ff1a5f68a7e3667ca14d3808f8a887626846043eb8b45a3ed5818f573",
  "updateLocationHealth": "f7a84aeb02ca7295ce9f9caf008c1a7d21a81ed416c74112a6e0c7270bdeb8f9",
  "evaluateAllCameras": "4fefb6396aa56413e02e7a073e4da2ca99619f0777d90c0ac31963a6c7212925",
  "markCameraPassed": "371cc77264a37750616d68dd41e82a31d5f2e682478fac728f29ad4d948b0f0f",
  "buildCameraGeometry": "2c23eac0a12428fbab7e0e0c846701784293dc344406650461d27b9096348bac",
  "updateApproachState": "231a02d9044530bf69db73496d0434c3fab53dd70dd3e2260f4fd0a46146383c",
  "computeAlertRequest": "16e668db83bdd1600d874b5f6acf84478dff93c6c95a2b608ed058ab94607ffd",
  "dispatchCameraAlerts": "4ecb323b8628cafcff2c8f766bcc60b1cee2ce8004f67c9b7eba56d72a8b3ee1",
  "alertRepeatIntervalMs": "9cf7b356afc58eb32b1277489109d7570d476db9be5a999e87bf34b525ab857e",
  "bridgeConfirmedCameraAlerts": "5c5ebcdf75ec1615a3ce38f0c428bb844098083add3b70088a5962f03d99b46a",
  "chooseUiCamera": "40b89a3c5dfe02154d43c08f17ce4756cfaaccf0f0d51a7ee73e494c399d666a",
  "clearUiCamera": "a835f7ab1a97fc82a4b6fff5b075d5f4119a18207df268796137daacf8515c90",
  "cleanupOldCameraStates": "9609523e6f530a88a90dea0adfcda7ed3119bbdac9eb3a64cb99c72fd6c9e3ee",
  "computeAdaptiveWarningDistanceM": "f18346d3b22098ca94612ec94893e2bb6bfed2a462c59ebf3b33c28e4528f389",
  "computeCameraAlertLevel": "fbde1b4fbc7e3b247028490e5cc567b07d27bf162c86df1185b953894bba6b35",
  "validDynamicLimit": "aef52b37013821b9ad3c6b79cc2e3048b03cbbae7af9a2296a8ff82cb4dbde8d",
  "handleOverspeed": "19d381e76cfe8a1994af14c9c216dc78f6dc5391af89c7fbd27ebbb2980d7eed"
}

LOCKED_CONSTANTS = {
    "LOCATION_STALE_MS": "8000L",
    "LOCATION_RECOVERY_RESET_MS": "3000L",
    "GPS_CAMERA_BRIDGE_START_MS": "1500L",
    "GPS_CAMERA_BRIDGE_MAX_MS": "12_000L",
    "FUSION_BRIDGE_MS": "12_000L",
    "SPEED_DISPLAY_STALE_MS": "4_000L",
    "SPEED_HARD_INVALIDATE_MS": "12_000L",
    "STATIONARY_RAW_SPEED_KMH": "2.5f",
    "STATIONARY_DERIVED_SPEED_KMH": "3.5f",
    "STATIONARY_CONFIRM_HITS": "2",
    "BEARING_HOLD_MS": "8000L",
    "LOCATION_FALLBACK_TRIGGER_MS": "6000L",
    "LOCATION_STALL_LOG_MS": "30_000L",
    "CAMERA_SEARCH_M": "2300f",
    "CAMERA_CONFIRM_EXTRA_M": "500f",
    "CAMERA_CLUSTER_BASE_M": "40f",
    "CAMERA_CLUSTER_EXTENDED_M": "110f",
    "PASS_BEHIND_DEG": "100f",
    "PASS_DISTANCE_GROWTH_M": "25f",
    "NEAR_PASS_AUDIT_M": "140f",
    "GLOBAL_ALERT_MIN_GAP_MS": "350L",
    "FIRST_ALERT_FORCE_MS": "1200L"
}

VETO_BEGIN = "// BEGIN V0.15 GUARDED NEGATIVE ROUTE VETO"
VETO_END = "// END V0.15 GUARDED NEGATIVE ROUTE VETO"


def extract_function(src: str, name: str) -> str:
    m = re.search(r"(?m)^\s*(?:private\s+|override\s+|@\w+\s+)*fun\s+" + re.escape(name) + r"\s*\(", src)
    if not m:
        raise ValueError(f"function not found: {name}")
    start = m.start()
    brace = src.find("{", m.end())
    eq = src.find("=", m.end(), min(len(src), m.end() + 250))
    if eq != -1 and (brace == -1 or eq < brace):
        brace = src.find("{", eq)
    if brace < 0:
        raise ValueError(f"body not found: {name}")
    depth = 0
    for i in range(brace, len(src)):
        ch = src[i]
        if ch == "{":
            depth += 1
        elif ch == "}":
            depth -= 1
            if depth == 0:
                return src[start:i + 1].strip()
    raise ValueError(f"unclosed body: {name}")


def normalize_allowed_extension(name: str, body: str) -> str:
    if name != "computeAlertRequest":
        return body

    if body.count(VETO_BEGIN) != 1 or body.count(VETO_END) != 1:
        raise ValueError("v0.15 guarded veto markers missing or duplicated")

    # The insertion is surrounded by the same blank line that existed in v0.12. Removing
    # this entire marked block must restore the original field-validated function byte-for-byte.
    pattern = re.compile(
        r"\n\n        // BEGIN V0\.15 GUARDED NEGATIVE ROUTE VETO\n.*?"
        r"        // END V0\.15 GUARDED NEGATIVE ROUTE VETO\n",
        re.S,
    )
    normalized, count = pattern.subn("\n\n", body)
    if count != 1:
        raise ValueError(f"unable to isolate guarded veto block (matches={count})")

    # Guard the intent of the allowed block too. Generic conflict/support can NEVER veto.
    block = body[body.index(VETO_BEGIN): body.index(VETO_END) + len(VETO_END)]
    required = [
        '!bridge && !state.warned',
        'explicitNegative?.verdict == "negative_route"',
        'explicitNegative.evidenceKind == "explicit_negative"',
        'CAMERA_NEGATIVE_ROUTE_SUPPRESSED',
        'return null',
    ]
    for token in required:
        if token not in block:
            raise ValueError(f"guarded veto lost required condition/token: {token}")
    forbidden = [
        'verdict == "conflict"',
        'verdict == "outside_coverage"',
        'verdict == "uncertain"',
        'verdict == "support"',
        'verdict == "weak_support"',
    ]
    for token in forbidden:
        if token in block:
            raise ValueError(f"generic road-geometry verdict illegally used by live veto: {token}")
    return normalized


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default="app/src/main/java/com/mojotech/mojodrive/LocationService.kt")
    args = ap.parse_args()
    src = Path(args.source).read_text(encoding="utf-8")

    failures = []
    for name, expected in EXPECTED.items():
        try:
            body = extract_function(src, name)
            body = normalize_allowed_extension(name, body)
        except Exception as e:
            failures.append(f"{name}: {e}")
            continue
        actual = hashlib.sha256(body.encode("utf-8")).hexdigest()
        if actual != expected:
            failures.append(f"{name}: locked baseline changed ({actual[:12]} != {expected[:12]})")

    for name, expected in LOCKED_CONSTANTS.items():
        m = re.search(r"private const val\s+" + re.escape(name) + r"\s*=\s*([^\n]+)", src)
        if not m:
            failures.append(f"constant {name}: missing")
            continue
        actual = m.group(1).strip()
        if actual != expected:
            failures.append(f"constant {name}: changed ({actual} != {expected})")

    if failures:
        print("BASELINE GUARD: FAIL")
        for f in failures:
            print(" -", f)
        print("\nThe field-validated v0.12 engine is locked. v0.15 permits only the marked explicit-negative FIRST-alert veto.")
        return 2

    print(
        f"BASELINE GUARD: PASS ({len(EXPECTED)} protected functions, including one normalized v0.15 veto extension, "
        f"+ {len(LOCKED_CONSTANTS)} constants unchanged)"
    )
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
