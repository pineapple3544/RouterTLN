"""Prepare the two original ROS streams, retaining scan and aligned odometry timestamps."""
from pathlib import Path
import csv,json
import numpy as np
from scipy.spatial import cKDTree
from PIL import Image
import yaml
from math import radians
D=Path(__file__).resolve().parent
ROOT=D.parents[1]
BASE=D.parent/'berlin-forward-holdout-100'
wp=json.loads((ROOT/'maps/berlin/global_waypoints.json').read_text())['global_traj_wpnts_iqp']['wpnts']
xy=np.array([[w['x_m'],w['y_m']] for w in wp])
ss=np.array([w['s_m'] for w in wp]); tree=cKDTree(xy)
def project(pos):
    k=tree.query(pos)[1]; ii=np.array([(k-1)%len(xy),k])
    jj=(ii+1)%len(xy); vec=xy[jj]-xy[ii];u=np.clip(((pos-xy[ii])*vec).sum(axis=1)/(vec*vec).sum(axis=1),0,1)
    diff=pos-xy[ii]-vec*u[:,None];j=np.argmin((diff*diff).sum(axis=1))
    dist=np.linalg.norm(vec[j])
    s=(ss[ii[j]]+u[j]*dist-ss[37])%(ss[-1]+np.linalg.norm(xy[0]-xy[-1]))
    if s>47:s-=ss[-1]+np.linalg.norm(xy[0]-xy[-1])
    return s,np.cross(vec[j],diff[j])/dist
def csvrows(p):return list(csv.DictReader(p.open()))
def nearest(ts,t):
    j=np.searchsorted(ts,t);return min((max(0,j-1),min(len(ts)-1,j)),key=lambda i:abs(ts[i]-t))
def prepare(group,seed,key):
    tag=f'random-{seed}';od=csvrows(BASE/group/(tag+'.odom.csv'));ms=csvrows(BASE/group/tag/(tag+'-1.csv'))
    ts=np.array([float(r['stamp']) for r in od]);first=nearest(ts,float(ms[0]['stamp']))
    od=od[first:];ts=ts[first:]
    ph=[[float(r[k]) for k in ('stamp','x','y','yaw')] for r in od]
    controls=[];curves=[]
    for m in ms:
        t=float(m['stamp']);idx=nearest(ts,t)
        oa=t+.001*(float(m['scan_source_age_ms'])-float(m['odom_source_age_ms']))
        oi=nearest(ts,oa)
        c=dict(scan=int(idx),odom=int(oi),stamp=t,steer=float(m['steer']),target=float(m['target_speed']),v=float(m['speed']),mode=int(m['mode']),side=int(m['side']),reason=m['reason'],stored=int(m['remembered_cells']),progress=float(m['progress_m']),latency=float(m['scan_source_age_ms'])/1000)
        controls.append(c)
        s,d=project(np.array([float(m['x']),float(m['y'])]));curves.append([s,d,ph[oi][3],c['steer'],c['v'],c['mode'],c['side'],t])
        if s>43 or (c['mode']==3 and seed==9006203):break
    boxes=next(z['boxes'] for f in (BASE/'offline').glob('results*.jsonl') for l in f.open() if (z:=json.loads(l))['case']==tag)
    ph=ph[:max(max(c['odom'],c['scan']) for c in controls)+3]
    payload=dict(physics=ph,controls=controls,boxes=boxes)
    (D/(key+'.json')).write_text(json.dumps(payload))
    return np.array(curves),payload
if __name__=='__main__':
    f,fd=prepare('ros',9006203,'failed')
    g,gd=prepare('ros-confirmation',9006203,'passed')
    b,bd=prepare('ros',5557202,'brake')
    # Interpolate continuous measurements onto a shared 1 cm progress grid.
    # Hold discrete states and published commands until the following decision.
    grid=np.arange(.01, min(f[-1,0],g[-1,0]),.01)
    def align(a):
        a=a[np.r_[True,np.diff(a[:,0])>1e-7]]
        ix=np.clip(np.searchsorted(a[:,0],grid,side='right')-1,0,len(a)-1)
        return np.column_stack([np.interp(grid,a[:,0],np.unwrap(a[:,i]) if i==2 else a[:,i]) if i in (1,2,4) else a[ix,i] for i in range(1,7)])
    a,z=align(f),align(g)
    dif=np.abs(a-z);dif[:,1]=np.abs(np.arctan2(np.sin(dif[:,1]),np.cos(dif[:,1])))
    np.savetxt(D/'spatial.csv',np.column_stack([grid,a,z]),delimiter=',',header='s,fail_d,fail_yaw,fail_steer,fail_v,fail_mode,fail_side,pass_d,pass_yaw,pass_steer,pass_v,pass_mode,pass_side',comments='')
    def intervals(mask,span=.15):
        start=np.flatnonzero(mask & ~np.r_[False,mask[:-1]]);end=np.flatnonzero(mask & ~np.r_[mask[1:],False])
        return [(round(grid[i],2),round(grid[j],2)) for i,j in zip(start,end) if j-i+1>=round(span/.01)]
    for i,th in enumerate([.03,radians(3),.05,.15,.5,.5]):
        print(i,intervals(dif[:,i]>th)[:25])
    for key,c in [('fail',fd),('pass',gd)]:
        print(key,[(r['progress'],r['mode'],r['side'],r['reason']) for j,r in enumerate(c['controls']) if j and (r['side'],r['mode'])!=(c['controls'][j-1]['side'],c['controls'][j-1]['mode'])])
    print('first original delta',next((i,fd['controls'][i],gd['controls'][i]) for i in range(min(len(f),len(g))) if abs(fd['controls'][i]['steer']-gd['controls'][i]['steer'])>1e-6))
