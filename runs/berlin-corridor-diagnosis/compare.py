"""Counterfactual replay of recorded Berlin lattice frames. Ground truth is analysis-only."""
import csv
import json
import math
import sys
from pathlib import Path
import numpy as np
import yaml
from PIL import Image

m=Path('maps/berlin')
meta=yaml.safe_load((m/'berlin.yaml').read_text())
base=(1-np.asarray(Image.open(m/'berlin.png').convert('L'))[::-1]/255)>=meta['free_thresh']
H,W=base.shape
res=meta['resolution']
x0,y0,_=meta['origin']
pts=json.loads((m/'global_waypoints.json').read_text())['global_traj_wpnts_iqp']['wpnts']
wp=np.array([[w['x_m'],w['y_m'],w['psi_rad']] for w in pts],dtype=np.float32)
if np.linalg.norm(wp[0,:2]-wp[-1,:2])<.05:wp=wp[:-1]
dx,dy=np.meshgrid(np.linspace(-.355,.355,7,dtype='f'),np.linspace(-.1666,.1666,5,dtype='f'))
dx,dy=dx.flatten(),dy.flatten()
def analyze(f,r):
    poses=next(yaml.safe_load_all(f.with_name(f.name.replace('.csv.corridor.jsonl','-poses.yaml')).read_text()))['poses']
    boxes=[(z['position']['x'],z['position']['y'],2*math.atan2(z['orientation']['z'],z['orientation']['w'])) for z in poses]
    def box_distance(x,y):
        ds=[]
        for bx,by,th in boxes:
            ux,uy=x-bx,y-by
            ds.append(np.hypot(np.maximum(0,abs(ux*np.cos(th)+uy*np.sin(th))-.15),
                               np.maximum(0,abs(-ux*np.sin(th)+uy*np.cos(th))-.15)))
        return np.min(ds,axis=0)
    cells=np.asarray(r['obstacle_cells'],dtype=int)
    dist=box_distance(x0+(cells%W+.5)*res,y0+(cells//W+.5)*res)
    def masks(wx,wy):
        ix=np.floor((wx-x0)/res).astype(int);iy=np.floor((wy-y0)/res).astype(int)
        idx=iy*W+ix;walls=base[iy,ix]
        return dict(full=walls|np.isin(idx,cells),near=walls|np.isin(idx,cells[dist<=res*.71+.015]),
                    truth=walls|(box_distance(wx,wy)<.000001),walls=walls),idx
    last={k:1<<r['origin'] for k in ['full','near','truth','walls']};dead={};critical=[];gap=0
    for i in range(1,32):
        cx,cy,h=wp[(r['waypoint']+2*i)%len(wp)]
        o=np.arange(-12,13,dtype='f')[:,None]*.1
        wx=cx-np.sin(h)*o+np.cos(h)*dx-np.sin(h)*dy
        wy=cy+np.cos(h)*o+np.sin(h)*dx+np.cos(h)*dy
        ms,idx=masks(wx,wy)
        for key,occupied in ms.items():
            available=~np.any(occupied,axis=1)
            free=sum((1<<j) for j in np.flatnonzero(available));a=last[key]
            nxt=(a | (a<<1) | (a>>1))&int(free)
            if key=='full':gap+=int(int(free)!=r['free'][i])
            if not nxt and key not in dead:dead[key]=i
            last[key]=nxt
        if i==r['first_empty']:
            a=r['reachable'][i-1]; cols=[j for j in range(25) if (a|a<<1|a>>1)&(1<<j)]
            for j in cols:
                hits=np.intersect1d(idx[j],cells)
                critical.append(dict(col=j-12,wall=int(np.any(ms['walls'][j])),points=[
                    [int(k),round(float(dist[np.where(cells==k)[0][0]]),3)] for k in hits]))
    return dict(step=r['step'],s=round(r['progress'],3),pose=r['pose'],origin=r['origin']-12,
                first_empty=r['first_empty'],dead=dead,mask_mismatches=gap,critical=critical,
                cells=len(cells),far_cells=int(sum(dist>.045)))
if __name__=='__main__':
    for name in sys.argv[1:]:
        f=Path(name);rows=[json.loads(l) for l in f.open() if l.endswith('}\n')]
        for r in rows:
            if r['first_empty']>0:print(f, json.dumps(analyze(f,r)))
