#!/usr/bin/env python3
"""Build MOJO Drive road-geometry shadow DB from proven field logs.

This does NOT alter camera decisions. It learns centerline-like approach traces only for
camera/direction passes that came within 160 m, so incomplete/abandoned branches do not
become trusted geometry. Multiple independently proven approaches may be retained for one
camera so curved/merged routes are not mistaken for conflicts.
"""
import argparse, csv, math, os, sqlite3, statistics, time
from collections import defaultdict

EARTH_R=6371000.0

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

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--camera-db',required=True)
    ap.add_argument('--output',required=True)
    ap.add_argument('logs',nargs='+')
    args=ap.parse_args()
    cams=load_cameras(args.camera_db)
    passes=defaultdict(list)
    # For each trip collect continuous candidate windows per camera.
    for log in args.logs:
        rows=[]
        with open(log,encoding='utf-8') as f:
            for r in csv.DictReader(f):
                if r['record_type'] not in ('LOCATION','LOCATION_ONLY') or not r['lat'] or not r['lon']:continue
                if r.get('location_valid','true').lower()!='true':continue
                lat=float(r['lat']);lon=float(r['lon']);t=int(r['wall_time_ms']); bear=float(r['bearing_deg']) if r.get('bearing_valid','').lower()=='true' and r.get('bearing_deg') else None
                rows.append((t,lat,lon,bear))
        active=defaultdict(list)
        last_t={}
        def flush(k):
            pts=active.pop(k,[])
            if len(pts)>=5:passes[k].append((os.path.basename(log),pts))
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
                active[k].append((t,lat,lon,bear,d));last_t[k]=t;touched.add(k)
            # windows disappear after 6 sec out of corridor
            for k in list(last_t):
                if k not in touched and t-last_t[k]>6000:flush(k)
        for k in list(active):flush(k)

    chosen=[]
    covered_keys=set()
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

        # Keep multiple proven near-pass approaches for one physical camera. A camera can be
        # reached through a curved/merged corridor on different trips. Shadow mode chooses
        # whichever learned polyline is closest to the current vehicle, so storing only one
        # 'best' pass would create false conflicts on another valid approach.
        accepted=0
        seen_signatures=[]
        for md,negspan,src,pts in sorted(good,key=lambda x:(x[0],x[1])):
            span=-negspan
            ds=[pts[0]]
            for q in pts[1:-1]:
                if dist_m(ds[-1][1],ds[-1][2],q[1],q[2])>=12:ds.append(q)
            ds.append(pts[-1])
            ds=rdp(ds,4.0)
            if len(ds)<2:continue
            travel=bearing_deg(ds[0][1],ds[0][2],ds[-1][1],ds[-1][2])
            mid=ds[len(ds)//2]
            # Suppress near-identical repeats while retaining genuinely different curves/merges.
            duplicate=False
            for old_b,old_lat,old_lon in seen_signatures:
                if angle_diff(travel,old_b)<=12 and dist_m(mid[1],mid[2],old_lat,old_lon)<=45:
                    duplicate=True;break
            if duplicate:continue
            conf='high' if md<=35 and span>=500 else ('medium' if md<=90 and span>=350 else 'low')
            chosen.append((k,src,md,span,travel,conf,ds))
            covered_keys.add(k)
            seen_signatures.append((travel,mid[1],mid[2]))
            accepted+=1
            if accepted>=4:break

    os.makedirs(os.path.dirname(os.path.abspath(args.output)),exist_ok=True)
    if os.path.exists(args.output):os.remove(args.output)
    con=sqlite3.connect(args.output)
    con.executescript('''
    PRAGMA journal_mode=OFF;
    CREATE TABLE meta(key TEXT PRIMARY KEY,value TEXT NOT NULL);
    CREATE TABLE road_segments(
      segment_id INTEGER PRIMARY KEY,
      camera_id TEXT NOT NULL,
      camera_type TEXT NOT NULL,
      road_name TEXT,
      source_log TEXT NOT NULL,
      source_kind TEXT NOT NULL,
      confidence TEXT NOT NULL,
      min_pass_distance_m REAL NOT NULL,
      route_span_m REAL NOT NULL,
      travel_bearing REAL NOT NULL,
      point_count INTEGER NOT NULL
    );
    CREATE INDEX idx_road_segments_camera ON road_segments(camera_id,camera_type);
    CREATE TABLE road_points(
      segment_id INTEGER NOT NULL,
      seq INTEGER NOT NULL,
      latitude REAL NOT NULL,
      longitude REAL NOT NULL,
      PRIMARY KEY(segment_id,seq)
    );
    CREATE INDEX idx_road_points_segment ON road_points(segment_id);
    ''')
    meta={
      'schema_version':'2', 'mode':'shadow_only', 'source_kind':'field_learned_stable_v014',
      'engine_effect':'false','policy':'conservative_multi_approach_v2','covered_cameras':str(len(covered_keys)),
      'build_epoch_ms':str(int(time.time()*1000)),
      'selection_rule':'same-direction, <=160m near-pass, >=220m route span, retain distinct proven approaches',
      'source_trip_count':str(len(args.logs))
    }
    con.executemany('insert into meta(key,value) values(?,?)',meta.items())
    for sid,(k,src,md,span,travel,conf,pts) in enumerate(sorted(chosen,key=lambda x:(x[0][0],x[0][1])),1):
        c=cams[k]
        con.execute('insert into road_segments values(?,?,?,?,?,?,?,?,?,?,?)',(sid,k[0],k[1],c['name'],src,'field_learned_stable_v014',conf,md,span,travel,len(pts)))
        con.executemany('insert into road_points values(?,?,?,?)',[(sid,i,p[1],p[2]) for i,p in enumerate(pts)])
    con.commit()
    print('covered cameras:',len(covered_keys),'segments:',len(chosen),'points:',con.execute('select count(*) from road_points').fetchone()[0])
    for r in con.execute('select camera_id,confidence,round(min_pass_distance_m,1),round(route_span_m,1),point_count from road_segments order by camera_id'):print(r)
    con.close()

if __name__=='__main__':main()
