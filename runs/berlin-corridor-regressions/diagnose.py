"""Align archived builds by distance and record the first disconnected DP row."""
import json, csv, math
from pathlib import Path
D=Path(__file__).resolve().parent
wp=json.loads((D.parents[1]/'maps/berlin/global_waypoints.json').read_text())['global_traj_wpnts_iqp']['wpnts']
def read(p):
    return [json.loads(l) for l in p.open()]
def local(r):
    x,y,yaw,*_=r['car']
    w=min(wp,key=lambda w:(w['x_m']-x)**2+(w['y_m']-y)**2)
    h=w['psi_rad']
    return dict(s=r['s'],lateral_m=-(x-w['x_m'])*math.sin(h)+(y-w['y_m'])*math.cos(h),
                heading_deg=math.degrees(math.remainder(yaw-h,2*math.pi)),target=r['gate'][2],steer=r['command'][0])
def count(a):return sum(n.bit_count() for n in a)
def records(folder):return {r['case']:r for p in folder.glob('results-*.jsonl') for r in read(p)}
out=[]
for p in sorted((D/'before87').glob('*.ticks')):
    tag=p.stem
    old=read(D/'before84'/p.name);new=read(p)
    fault=next(r for r in new if r['reason']=='no_corridor')
    i=fault['first_empty'];progress=fault['s']
    ref84=records(D.parent/'berlin-forward-holdout-100/offline')[tag]
    ref87=records(D.parent/'berlin-sync-braking/offline-v1')[tag]
    for folder,ref in [('before84',ref84),('before87',ref87)]:
        current=records(D/folder)[tag]
        assert current['laps']==ref['laps'] and current['first_rejection']==ref['first_rejection']
    sections=[]
    for delta in (2.5,1.5,.85,.35,.04,0):
        at=progress-delta
        a,b=(min(rr,key=lambda z:abs(z['s']-at)) for rr in (old,new))
        sections.append({'at':at,'v84':local(a),'v87':local(b)})
    out.append(dict(layout=tag,lap=len(ref87['laps'])+1,first_stop_m=progress,stage=i,
                    free_at_empty=count(fault['free_columns'][i]),reachable_before=count(fault['reachable_columns'][i-1]),
                    reachable_at_empty=count(fault['reachable_columns'][i]),waypoint=fault['waypoint'],
                    map_left_m=wp[fault['waypoint']]['d_left'],map_right_m=wp[fault['waypoint']]['d_right'],
                    obstacle_cells=fault['occupancy'],around_failure=sections))
(D/'failures.json').write_text(json.dumps(out,indent=2)+'\n')
