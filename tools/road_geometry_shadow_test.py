#!/usr/bin/env python3
"""Regression test for MVP 0.15 road-geometry shadow policy.

Tests both the v0.14 conservative positive-road behavior and the new v0.15 explicit
negative-route evidence. Generic geometry remains SHADOW ONLY; only the explicit negative
route verdict is eligible for the separately guarded first-alert veto.
"""
from __future__ import annotations
import json, math, sqlite3
from pathlib import Path

SUPPORT_DISTANCE_M=70.0
WEAK_SUPPORT_DISTANCE_M=160.0
COVERAGE_DISTANCE_M=180.0
SUPPORT_HEADING_DEG=50.0
WEAK_SUPPORT_HEADING_DEG=65.0
CONFLICT_HEADING_DEG=105.0
NEGATIVE_ROUTE_DISTANCE_M=85.0
NEGATIVE_ROUTE_HEADING_DEG=60.0
POLICY="conservative_multi_approach_negative_route_v3"

ROOT=Path(__file__).resolve().parents[1]
DB=ROOT/"app/src/main/assets/road_geometry.sqlite"
CASES=ROOT/"tools/fixtures/v015_shadow_cases.json"


def angle_diff(a,b):
    d=abs(a-b)%360.0
    return 360.0-d if d>180.0 else d


def nearest(lat,lon,points):
    if len(points)<2:return None
    mx=111320.0*math.cos(math.radians(lat)); my=110540.0
    best=(float('inf'),0.0)
    for a,b in zip(points,points[1:]):
        ax=(a[1]-lon)*mx; ay=(a[0]-lat)*my; bx=(b[1]-lon)*mx; by=(b[0]-lat)*my
        vx=bx-ax; vy=by-ay; vv=vx*vx+vy*vy
        if vv<1e-6:continue
        t=max(0.0,min(1.0,-(ax*vx+ay*vy)/vv)); qx=ax+t*vx; qy=ay+t*vy
        d=math.hypot(qx,qy)
        if d<best[0]: best=(d,math.degrees(math.atan2(vx,vy))%360.0)
    return None if not math.isfinite(best[0]) else best


def classify_positive(d,h,bearing_available=True):
    if d>COVERAGE_DISTANCE_M:return 'outside_coverage'
    if not bearing_available:return 'position_only'
    if d<=SUPPORT_DISTANCE_M and h<=SUPPORT_HEADING_DEG:return 'support'
    if d<=WEAK_SUPPORT_DISTANCE_M and h<=WEAK_SUPPORT_HEADING_DEG:return 'weak_support'
    if h>=CONFLICT_HEADING_DEG:return 'conflict'
    return 'uncertain'


def load_db():
    con=sqlite3.connect(DB)
    try:
        assert con.execute('pragma integrity_check').fetchone()[0]=='ok'
        meta=dict(con.execute('select key,value from meta'))
        pos={}; neg={}
        for sid,cid,typ,src,conf in con.execute('select segment_id,camera_id,camera_type,source_kind,confidence from road_segments order by segment_id'):
            pts=[(lat,lon) for _,lat,lon in con.execute('select seq,latitude,longitude from road_points where segment_id=? order by seq',(sid,))]
            pos.setdefault((str(cid),typ),[]).append(dict(id=sid,source=src,confidence=conf,points=pts))
        for sid,cid,typ,src,conf,label in con.execute('select exclusion_id,camera_id,camera_type,source_kind,confidence,label from exclusion_segments order by exclusion_id'):
            pts=[(lat,lon) for _,lat,lon in con.execute('select seq,latitude,longitude from exclusion_points where exclusion_id=? order by seq',(sid,))]
            neg.setdefault((str(cid),typ),[]).append(dict(id=sid,source=src,confidence=conf,label=label,points=pts))
        return meta,pos,neg
    finally:con.close()


def evaluate(pos,neg,cid,typ,lat,lon,bearing):
    positives=[]; negatives=[]
    for s in pos.get((cid,typ),[]):
        n=nearest(lat,lon,s['points']);
        if n is None:continue
        d,b=n; h=angle_diff(bearing,b) if bearing is not None else 180.0
        positives.append({**s,'distance':d,'delta':h,'verdict':classify_positive(d,h,bearing is not None),'kind':'positive'})
    for s in neg.get((cid,typ),[]):
        n=nearest(lat,lon,s['points']);
        if n is None:continue
        d,b=n; h=angle_diff(bearing,b) if bearing is not None else 180.0
        negatives.append({**s,'distance':d,'delta':h,'kind':'negative'})

    def choose_positive(v):
        xs=[x for x in positives if x['verdict']==v]
        return min(xs,key=lambda x:x['distance']+min(x['delta'],180)*.35) if xs else None

    strong=choose_positive('support') or choose_positive('weak_support')
    if strong:return strong

    if bearing is not None:
        nm=[x for x in negatives if x['distance']<=NEGATIVE_ROUTE_DISTANCE_M and x['delta']<=NEGATIVE_ROUTE_HEADING_DEG]
        if nm:
            x=min(nm,key=lambda x:x['distance']+x['delta']*.35).copy(); x['verdict']='negative_route'; return x

    fallback=choose_positive('uncertain') or choose_positive('position_only') or choose_positive('conflict')
    if fallback:return fallback
    if positives:return min(positives,key=lambda x:x['distance'])
    if negatives:
        x=min(negatives,key=lambda x:x['distance']).copy(); x['verdict']='outside_coverage'; return x
    return None


def main():
    meta,pos,neg=load_db(); failures=[]
    if meta.get('engine_effect')!='explicit_negative_only':failures.append('engine_effect must be explicit_negative_only')
    if meta.get('mode')!='guarded_negative_veto':failures.append('mode must be guarded_negative_veto')
    if meta.get('generic_geometry_engine_effect')!='false':failures.append('generic geometry must remain shadow-only')
    if meta.get('explicit_negative_veto')!='true':failures.append('explicit negative veto must be enabled')
    if meta.get('policy')!=POLICY:failures.append(f"policy={meta.get('policy')} expected={POLICY}")
    if len(neg.get(('14550','speed'),[]))<2:failures.append('14550 requires two explicit negative-route segments')

    cases=json.loads(CASES.read_text(encoding='utf-8'))['cases']
    print(f'ROAD GEOMETRY SHADOW TEST: policy={POLICY} cases={len(cases)}')
    for c in cases:
        x=evaluate(pos,neg,str(c['camera_id']),c['camera_type'],float(c['lat']),float(c['lon']),None if c.get('bearing') is None else float(c['bearing']))
        if x is None:
            failures.append(c['name']+': no geometry'); print('  FAIL',c['name'],': no geometry'); continue
        ok=x['verdict'] in c['allowed']
        print(f"  {'PASS' if ok else 'FAIL'} {c['name']}: {x['verdict']} d={x['distance']:.1f}m headingDelta={x['delta']:.1f}° kind={x['kind']} segment={x['id']}")
        if not ok:failures.append(f"{c['name']}: {x['verdict']} not in {c['allowed']}")
    if failures:
        print('ROAD GEOMETRY SHADOW TEST: FAIL')
        for f in failures:print(' -',f)
        return 2
    print('ROAD GEOMETRY SHADOW TEST: PASS')
    return 0

if __name__=='__main__':raise SystemExit(main())
