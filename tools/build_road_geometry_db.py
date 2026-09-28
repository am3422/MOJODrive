#!/usr/bin/env python3
"""Build MOJO Drive road-geometry shadow DB from proven field logs.

Positive geometry is learned only from same-direction passes that actually come within
160 m of a camera. Explicit negative geometry is learned only from human/field-confirmed
false-route labels supplied via --negative-labels.

Generic positive/conflict geometry remains SHADOW ONLY. MVP 0.15 allows only an explicitly
labeled negative route to veto a FIRST live alert through a separately guarded runtime path.
Any log used as explicit negative evidence is automatically excluded from positive learning
for this DB build, preventing one ground-truth-negative trip from self-training positives.
"""
from __future__ import annotations
import argparse, csv, json, math, os, sqlite3, time
from collections import defaultdict

EARTH_R = 6371000.0


def dist_m(a,b,c,d):
    p1=math.radians(a); p2=math.radians(c); dp=p2-p1; dl=math.radians(d-b)
    x=math.sin(dp/2)**2+math.cos(p1)*math.cos(p2)*math.sin(dl/2)**2
    return 2*EARTH_R*math.atan2(math.sqrt(x),math.sqrt(max(0.0,1.0-x)))


def bearing_deg(a,b,c,d):
    p1=math.radians(a); p2=math.radians(c); dl=math.radians(d-b)
    y=math.sin(dl)*math.cos(p2); x=math.cos(p1)*math.sin(p2)-math.sin(p1)*math.cos(p2)*math.cos(dl)
    return math.degrees(math.atan2(y,x))%360.0


def angle_diff(a,b):
    x=abs(a-b)%360.0
    return 360.0-x if x>180 else x


def rdp(points, eps_m):
    if len(points)<=2:return points
    lat0=sum(p[1] for p in points)/len(points); kx=111320.0*math.cos(math.radians(lat0)); ky=110540.0
    def xy(p): return (p[2]*kx,p[1]*ky)
    A=xy(points[0]); B=xy(points[-1]); vx=B[0]-A[0]; vy=B[1]-A[1]; vv=vx*vx+vy*vy
    best_i=-1; best=-1.0
    for i,p in enumerate(points[1:-1],1):
        P=xy(p)
        if vv<=1e-9:d=math.hypot(P[0]-A[0],P[1]-A[1])
        else:
            t=max(0,min(1,((P[0]-A[0])*vx+(P[1]-A[1])*vy)/vv)); Q=(A[0]+t*vx,A[1]+t*vy); d=math.hypot(P[0]-Q[0],P[1]-Q[1])
        if d>best:best=d;best_i=i
    if best>eps_m:
        L=rdp(points[:best_i+1],eps_m); R=rdp(points[best_i:],eps_m); return L[:-1]+R
    return [points[0],points[-1]]


def load_cameras(db):
    con=sqlite3.connect(db)
    rows=con.execute("SELECT camera_id,camera_type,latitude,longitude,road_bearing,oneway,direction_confidence,road_name FROM cameras").fetchall(); con.close()
    out={}
    for cid,typ,lat,lon,rb,one,conf,name in rows:
        out[(str(cid),typ)]={'lat':lat,'lon':lon,'rb':rb,'oneway':bool(one),'conf':(conf or 'unknown').lower(),'name':name or ''}
    return out


def load_location_rows(log):
    rows=[]
    with open(log,encoding='utf-8') as f:
        for r in csv.DictReader(f):
            if r['record_type'] not in ('LOCATION','LOCATION_ONLY') or not r['lat'] or not r['lon']:continue
            if r.get('location_valid','true').lower()!='true':continue
            lat=float(r['lat']); lon=float(r['lon']); t=int(r['wall_time_ms'])
            bear=float(r['bearing_deg']) if r.get('bearing_valid','').lower()=='true' and r.get('bearing_deg') else None
            rows.append((t,lat,lon,bear))
    return rows


def simplify_trace(rows, min_step_m=8.0, rdp_m=3.0):
    if len(rows)<2:return []
    ds=[rows[0]]
    for p in rows[1:-1]:
        if dist_m(ds[-1][1],ds[-1][2],p[1],p[2])>=min_step_m:ds.append(p)
    ds.append(rows[-1])
    return rdp(ds,rdp_m)


def build_positive(cams, rows_by_log):
    passes=defaultdict(list)
    for log_name,rows in rows_by_log.items():
        active=defaultdict(list); last_t={}
        def flush(k):
            pts=active.pop(k,[])
            if len(pts)>=5:passes[k].append((log_name,pts))
            last_t.pop(k,None)
        for t,lat,lon,bear in rows:
            touched=set()
            for k,c in cams.items():
                d=dist_m(lat,lon,c['lat'],c['lon'])
                if d>1050:continue
                if bear is not None and c['rb'] is not None:
                    directed=c['oneway'] and c['conf'] in ('high','medium')
                    dd=angle_diff(bear,c['rb']) if directed else min(angle_diff(bear,c['rb']),angle_diff(bear,(c['rb']+180)%360))
                    if dd>60:continue
                if k in last_t and t-last_t[k]>5000:flush(k)
                active[k].append((t,lat,lon,bear,d)); last_t[k]=t; touched.add(k)
            for k in list(last_t):
                if k not in touched and t-last_t[k]>6000:flush(k)
        for k in list(active):flush(k)

    chosen=[]; covered=set()
    for k,groups in passes.items():
        good=[]
        for src,pts in groups:
            md=min(p[4] for p in pts)
            if md>160:continue
            pts=[p for p in pts if p[4]<=950]
            if len(pts)<5:continue
            span=sum(dist_m(a[1],a[2],b[1],b[2]) for a,b in zip(pts,pts[1:]))
            if span<220:continue
            good.append((md,-span,src,pts))
        if not good:continue
        accepted=0; seen=[]
        for md,negspan,src,pts in sorted(good,key=lambda x:(x[0],x[1])):
            span=-negspan
            simp=simplify_trace(pts,12.0,4.0)
            if len(simp)<2:continue
            travel=bearing_deg(simp[0][1],simp[0][2],simp[-1][1],simp[-1][2])
            mid=simp[len(simp)//2]
            if any(angle_diff(travel,ob)<=12 and dist_m(mid[1],mid[2],olat,olon)<=45 for ob,olat,olon in seen):continue
            conf='high' if md<=35 and span>=500 else ('medium' if md<=90 and span>=350 else 'low')
            chosen.append((k,src,md,span,travel,conf,simp)); covered.add(k); seen.append((travel,mid[1],mid[2])); accepted+=1
            if accepted>=4:break
    return chosen,covered


def build_negative(cams, rows_by_log, label_file):
    if not label_file:return [],set(),0
    data=json.load(open(label_file,encoding='utf-8'))
    labels=data.get('labels',[]); chosen=[]; covered=set()
    for item in labels:
        k=(str(item['camera_id']),item['camera_type'])
        if k not in cams: raise RuntimeError(f"negative label camera not in DB: {k}")
        src=os.path.basename(item['source_log'])
        rows=rows_by_log.get(src)
        if rows is None: raise RuntimeError(f"negative label source log missing from inputs: {src}")
        a=int(item['start_wall_time_ms']); b=int(item['end_wall_time_ms'])
        window=[p for p in rows if a<=p[0]<=b]
        if len(window)<5: raise RuntimeError(f"negative label {item['label']} has only {len(window)} points")
        simp=simplify_trace(window,8.0,3.0)
        span=sum(dist_m(x[1],x[2],y[1],y[2]) for x,y in zip(simp,simp[1:]))
        if span<60: raise RuntimeError(f"negative label {item['label']} span too short: {span:.1f}m")
        chosen.append((k,src,item.get('confidence','high'),item['label'],item.get('reason','field_confirmed_false_route'),span,simp))
        covered.add(k)
    return chosen,covered,len(labels)


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--camera-db',required=True)
    ap.add_argument('--output',required=True)
    ap.add_argument('--negative-labels')
    ap.add_argument('logs',nargs='+')
    args=ap.parse_args()
    cams=load_cameras(args.camera_db)
    rows_by_log={os.path.basename(p):load_location_rows(p) for p in args.logs}

    negative_source_logs=set()
    if args.negative_labels:
        negative_label_data=json.load(open(args.negative_labels,encoding='utf-8'))
        negative_source_logs={
            os.path.basename(item['source_log'])
            for item in negative_label_data.get('labels',[])
        }

    positive_rows_by_log={
        name: rows
        for name,rows in rows_by_log.items()
        if name not in negative_source_logs
    }

    positive,positive_covered=build_positive(cams,positive_rows_by_log)
    negative,negative_covered,negative_label_count=build_negative(cams,rows_by_log,args.negative_labels)

    os.makedirs(os.path.dirname(os.path.abspath(args.output)),exist_ok=True)
    if os.path.exists(args.output):os.remove(args.output)
    con=sqlite3.connect(args.output)
    con.executescript('''
    PRAGMA journal_mode=OFF;
    CREATE TABLE meta(key TEXT PRIMARY KEY,value TEXT NOT NULL);
    CREATE TABLE road_segments(
      segment_id INTEGER PRIMARY KEY,camera_id TEXT NOT NULL,camera_type TEXT NOT NULL,road_name TEXT,
      source_log TEXT NOT NULL,source_kind TEXT NOT NULL,confidence TEXT NOT NULL,
      min_pass_distance_m REAL NOT NULL,route_span_m REAL NOT NULL,travel_bearing REAL NOT NULL,point_count INTEGER NOT NULL);
    CREATE INDEX idx_road_segments_camera ON road_segments(camera_id,camera_type);
    CREATE TABLE road_points(segment_id INTEGER NOT NULL,seq INTEGER NOT NULL,latitude REAL NOT NULL,longitude REAL NOT NULL,PRIMARY KEY(segment_id,seq));
    CREATE INDEX idx_road_points_segment ON road_points(segment_id);
    CREATE TABLE exclusion_segments(
      exclusion_id INTEGER PRIMARY KEY,camera_id TEXT NOT NULL,camera_type TEXT NOT NULL,road_name TEXT,
      source_log TEXT NOT NULL,source_kind TEXT NOT NULL,confidence TEXT NOT NULL,label TEXT NOT NULL,
      reason TEXT NOT NULL,route_span_m REAL NOT NULL,point_count INTEGER NOT NULL);
    CREATE INDEX idx_exclusion_segments_camera ON exclusion_segments(camera_id,camera_type);
    CREATE TABLE exclusion_points(exclusion_id INTEGER NOT NULL,seq INTEGER NOT NULL,latitude REAL NOT NULL,longitude REAL NOT NULL,PRIMARY KEY(exclusion_id,seq));
    CREATE INDEX idx_exclusion_points_segment ON exclusion_points(exclusion_id);
    ''')
    meta={
      'schema_version':'3','mode':'guarded_negative_veto','source_kind':'field_learned_stable_v015','engine_effect':'explicit_negative_only',
      'policy':'conservative_multi_approach_negative_route_v3','covered_cameras':str(len(positive_covered)),
      'exclusion_covered_cameras':str(len(negative_covered)),
      'negative_route_labels':str(negative_label_count),
      'exclusion_segments':str(len(negative)),
      'source_trip_count':str(len(rows_by_log)),
      'positive_source_trip_count':str(len(positive_rows_by_log)),
      'negative_source_trip_count':str(len(negative_source_logs)),
      'build_epoch_ms':str(int(time.time()*1000)),
      'generic_geometry_engine_effect':'false','explicit_negative_veto':'true',
      'selection_rule':'positive <=160m proven pass stays shadow-only; negative explicit field-confirmed false-route labels only may veto first live alert'}
    con.executemany('insert into meta(key,value) values(?,?)',meta.items())
    for sid,(k,src,md,span,travel,conf,pts) in enumerate(sorted(positive,key=lambda x:(x[0][0],x[0][1])),1):
        c=cams[k]
        con.execute('insert into road_segments values(?,?,?,?,?,?,?,?,?,?,?)',(sid,k[0],k[1],c['name'],src,'field_learned_stable_v015',conf,md,span,travel,len(pts)))
        con.executemany('insert into road_points values(?,?,?,?)',[(sid,i,p[1],p[2]) for i,p in enumerate(pts)])
    for eid,(k,src,conf,label,reason,span,pts) in enumerate(negative,1):
        c=cams[k]
        con.execute('insert into exclusion_segments values(?,?,?,?,?,?,?,?,?,?,?)',(eid,k[0],k[1],c['name'],src,'user_confirmed_negative_route_v015',conf,label,reason,span,len(pts)))
        con.executemany('insert into exclusion_points values(?,?,?,?)',[(eid,i,p[1],p[2]) for i,p in enumerate(pts)])
    con.commit()
    print('integrity:',con.execute('pragma integrity_check').fetchone()[0])
    print('positive cameras:',len(positive_covered),'segments:',len(positive),'points:',con.execute('select count(*) from road_points').fetchone()[0])
    print('negative cameras:',len(negative_covered),'segments:',len(negative),'points:',con.execute('select count(*) from exclusion_points').fetchone()[0])
    con.close()

if __name__=='__main__':main()
