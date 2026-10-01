#!/usr/bin/env python3
import hashlib, math, sqlite3, sys
from pathlib import Path

DB = Path("app/src/main/assets/shiraz_cameras.sqlite")
EXPECTED_BASE_DIGEST = "e5c38e0fcae5b76cbcb692d2c4540e0293b0352cb92095f15f409e5657751fdc"
EXPECTED_TOTAL = 481
EXPECTED_FARS = 214
EXPECTED_DIRECTED_FARS = 28

COLS = (
    "camera_id,camera_type,latitude,longitude,speed_limit,road_name,road_highway,"
    "road_distance_m,speed_source,speed_confidence,road_bearing,oneway,"
    "direction_mode,direction_confidence,road_layer"
)

def adiff(a, b):
    return abs((a - b + 180.0) % 360.0 - 180.0)

def fail(msg):
    print("FARS DIRECTION DB: FAIL -", msg)
    raise SystemExit(2)

def main():
    con = sqlite3.connect(DB)
    if con.execute("PRAGMA integrity_check").fetchone()[0] != "ok": fail("SQLite integrity")
    total = con.execute("SELECT COUNT(*) FROM cameras").fetchone()[0]
    fars = con.execute("SELECT COUNT(*) FROM cameras WHERE camera_id LIKE 'FARS_%'").fetchone()[0]
    directed = con.execute("SELECT COUNT(*) FROM cameras WHERE camera_id LIKE 'FARS_%' AND oneway=1 AND road_bearing IS NOT NULL AND direction_confidence IN ('high','medium')").fetchone()[0]
    if total != EXPECTED_TOTAL: fail(f"total {total} != {EXPECTED_TOTAL}")
    if fars != EXPECTED_FARS: fail(f"FARS count {fars} != {EXPECTED_FARS}")
    if directed != EXPECTED_DIRECTED_FARS: fail(f"directed FARS {directed} != {EXPECTED_DIRECTED_FARS}")

    # Stable 267 rows must remain byte-value identical to the original 0.15 database.
    base = con.execute(f"SELECT {COLS} FROM cameras WHERE camera_id NOT LIKE 'FARS_%' ORDER BY camera_id,camera_type").fetchall()
    digest = hashlib.sha256(repr(base).encode()).hexdigest()
    if len(base) != 267 or digest != EXPECTED_BASE_DIGEST:
        fail(f"stable baseline rows changed: count={len(base)} digest={digest}")

    # Low-metadata province rows are excluded by a general data-quality rule.
    bad = con.execute("SELECT COUNT(*) FROM cameras WHERE camera_id LIKE 'FARS_%' AND (TRIM(COALESCE(road_name,''))='' OR speed_limit IS NULL OR road_distance_m IS NULL)").fetchone()[0]
    if bad != 0: fail(f"low-metadata FARS rows still active: {bad}")

    # Known opposite-carriageway pairs from the 2026-09-30 field route must remain split.
    pairs = [('FARS_7943','FARS_7953'), ('FARS_5251','FARS_5268'), ('FARS_8569','FARS_8610')]
    for a,b in pairs:
        ra=con.execute("SELECT road_bearing,oneway,direction_confidence FROM cameras WHERE camera_id=?",(a,)).fetchone()
        rb=con.execute("SELECT road_bearing,oneway,direction_confidence FROM cameras WHERE camera_id=?",(b,)).fetchone()
        if not ra or not rb or ra[0] is None or rb[0] is None: fail(f"missing directed pair {a}/{b}")
        if ra[1] != 1 or rb[1] != 1: fail(f"pair not directed {a}/{b}")
        if adiff(ra[0],rb[0]) < 150: fail(f"pair bearings not opposite {a}/{b}: {ra[0]}/{rb[0]}")

    # Reverse-carriageway cameras that falsely alerted outbound now carry reliable reverse bearings.
    for cid in ['FARS_6552','FARS_3429','FARS_4560','FARS_5268','FARS_3695','FARS_10553']:
        r=con.execute("SELECT road_bearing,oneway,direction_confidence FROM cameras WHERE camera_id=?",(cid,)).fetchone()
        if not r or r[0] is None or r[1] != 1 or r[2] not in ('high','medium'): fail(f"missing reliable direction for {cid}")

    con.close()
    print(f"FARS DIRECTION DB: PASS total={total} stable=267 fars={fars} directed_fars={directed}; opposite pairs separated")

if __name__ == '__main__': main()
