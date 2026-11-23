"""Audit matching failures, static map and box proximity, and post-fix trajectories."""
from pathlib import Path
import csv,sys,json,math,hashlib
import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
import yaml
D=Path(__file__).resolve().parent
M=D.parents[1]/'maps/berlin'
base=D.parent/'berlin-corridor-regressions/unwind-100'
def records(folder):
    return {r['case']:r for p in folder.glob('results*.jsonl') for l in p.open() if (r:=json.loads(l))}
old=records(base); replay=records(D/'before'); after=records(D/sys.argv[1])
meta=yaml.safe_load((M/'berlin.yaml').read_text()); m=np.asarray(Image.open(M/meta['image']).convert('L'))[::-1]<205
res=meta['resolution'];wall_distance=distance_transform_edt(~m)*res
wp=json.loads((M/'global_waypoints.json').read_text())['global_traj_wpnts_iqp']['wpnts']
diagnoses=[]
for file in sorted((D/'before').glob('*.ticks')):
    tag=file.stem
    assert replay[tag]['first_rejection']==old[tag]['first_rejection'] and replay[tag]['laps']==old[tag]['laps']
    original=[json.loads(l) for l in file.open()]
    fix=[json.loads(l) for l in (D/sys.argv[1]/file.name).open()]
    r=next(z for z in original if z['reason']=='no_corridor'); i=r['first_empty']
    def count(stage):return sum(x.bit_count() for x in stage)
    ids=np.array(r['occupancy']);x,y=ids%m.shape[1],ids//m.shape[1]
    xy=np.array(meta['origin'][:2])+(np.stack([x,y],axis=-1)+.5)*res
    nearby=np.linalg.norm(xy-r['car'][:2],axis=1)<1.5
    xy=xy[nearby];x=x[nearby];y=y[nearby]
    dist=np.full(len(x),np.inf)
    for xx,yy,a,L,W in old[tag]['boxes']:
        c,s=np.cos(a),np.sin(a);v=(xy-[xx,yy])@np.array([[c,-s],[s,c]])
        dist=np.minimum(dist,np.linalg.norm(np.maximum(abs(v)-[L/2,W/2],0),axis=1))
    sample=[]
    for step in (2,1,.5,.15,0):
        at=r['s']-step
        pair={}
        for name,rows in [('before',original),('after',fix)]:
            z=min(rows,key=lambda t:abs(t['s']-at));xx,yy,h,*_=z['car']
            w=min(wp,key=lambda w:(xx-w['x_m'])**2+(yy-w['y_m'])**2)
            pair[name]=dict(progress=z['s'],heading_to_reference_deg=math.degrees(math.remainder(h-w['psi_rad'],math.tau)),lateral_m=-(xx-w['x_m'])*math.sin(w['psi_rad'])+(yy-w['y_m'])*math.cos(w['psi_rad']),gate=z['gate'],command=z['command'])
        sample.append(pair)
    diagnoses.append(dict(seed=tag,step=r['step'],progress=r['s'],lap=len(old[tag]['laps'])+1,stage=i,
        reachable_previous=count(r['reachable_columns'][i-1]),reachable_current=count(r['reachable_columns'][i]),free_current=count(r['free_columns'][i]),
        near_cells=len(x),near_box=int(sum(dist<.06)),other_cells_near_wall=int(sum((dist>=.06)&(wall_distance[y,x]<=.08))),
        unexplained=int(sum((dist>=.06)&(wall_distance[y,x]>.08))),before_after=sample,
        curved_entry_ticks=sum(z['subdivisions']==0 for z in fix),laps=[z['seconds'] for z in after[tag]['laps']]))
(D/'diagnosis.json').write_text(json.dumps(diagnoses,indent=2)+'\n')
