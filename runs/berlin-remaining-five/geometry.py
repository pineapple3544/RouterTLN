from pathlib import Path
import numpy as np, json, math, yaml
from PIL import Image
D=Path(__file__).resolve().parent; M=D.parents[1]/'maps/berlin'
meta=yaml.safe_load((M/'berlin.yaml').read_text()); base=np.array(Image.open(M/meta['image']).convert('L'))[::-1]<205
wp=json.loads((M/'global_waypoints.json').read_text())['global_traj_wpnts_iqp']['wpnts'][:-1]
xy=np.array([[w['x_m'],w['y_m']] for w in wp]); yaw=np.array([w['psi_rad'] for w in wp]); cells=np.array([[x,y] for x in np.linspace(-.355,.355,7) for y in np.linspace(-.1666,.1666,5)])
def check(poses,head,occ):
    c,s=np.cos(head),np.sin(head); g=np.stack((poses[...,0,None]+c[...,None]*cells[:,0]-s[...,None]*cells[:,1],poses[...,1,None]+s[...,None]*cells[:,0]+c[...,None]*cells[:,1]),axis=-1)
    ix=((g-meta['origin'][:2])/.04).astype(int);return ~np.any(occ[ix[...,1],ix[...,0]],axis=-1)
def run():
    findings=[]
    for f in sorted((D/'before').glob('*.ticks')):
        rr=[json.loads(l) for l in f.open()];k=next(i for i,z in enumerate(rr) if z['reason']=='no_corridor');r=rr[k]; prev=rr[k-1]; w=r['waypoint'];car=np.array(r['car'][:2]);h=r['car'][2];occ=base.copy();occ.ravel() # flipped array copied
        occ.flat[np.array(r['occupancy'])]=1
        prev_w=np.argmin(((xy-prev['car'][:2])**2).sum(axis=1));old=np.array(prev['planned_offsets'])
        W=(w+np.arange(63))%len(wp);normal=np.stack((-np.sin(yaw[W]),np.cos(yaw[W])),axis=-1);ref=xy[W]
        arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(ref,axis=0),axis=1))]
        lat=(car-ref[0])@normal[0];along=(car-ref[0])@[math.cos(yaw[w]),math.sin(yaw[w])]
        arc-=along
        slope=np.tan(np.clip(math.remainder(h-yaw[w],math.tau),-1.15,1.15))
        previous=np.interp(np.arange(63)+((w-prev_w+len(wp)//2)%len(wp)-len(wp)//2),np.arange(32)*2,old)
        tests=[]
        for length in [.5,.8,1.,1.3,1.6,2.,2.5]:
            ts=np.clip(arc/length,0,1)
            for target in np.arange(-1.2,1.21,.05):
                ease=ts*ts*(3-2*ts);offset=lat*(1-ease)+target*ease+length*slope*ts*(1-ts)**2
                t=np.clip((arc-max(1.8,length)) / 1.1,0,1);t=t*t*(3-2*t)
                offset=offset*(1-t)+previous*t
                p=ref+normal*offset[:,None];p[0]=car
                p=np.stack([np.interp(np.arange(125)*.5,np.arange(63),p[:,i]) for i in (0,1)],axis=-1)
                diff=np.gradient(p,axis=0);head=np.arctan2(diff[:,1],diff[:,0]);head[0]=h
                ok=check(p,head,occ);valid=bool(np.all(ok))
                if valid:tests.append([round(target,2),length,round(np.max(np.abs(np.diff(np.unwrap(head))/np.maximum(.01,np.linalg.norm(np.diff(p,axis=0),axis=1)))[:60]),2)])
        before=prev['planned_offsets'];e=r['first_empty'];occc=[sum(x.bit_count() for x in arr) for arr in [r['reachable_columns'][e-1],r['free_columns'][e]]]
        findings.append(dict(case=f.stem,progress=r['s'],lap=1+int(r['s']/48.784),stage=e,reachable_before=occc[0],open_cells=occc[1],feasible=len(tests),best=sorted(tests,key=lambda x:x[-1])[:4]))
    print(json.dumps(findings,indent=2));(D/'geometry.json').write_text(json.dumps(findings,indent=2))
if __name__=='__main__':run()
