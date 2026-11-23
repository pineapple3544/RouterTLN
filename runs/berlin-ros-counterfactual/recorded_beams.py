"""Reconstruct the collision-beam test at the original odometry poses."""
from brake_variants import *
dat=json.loads((D/'brake.json').read_text());frames=np.array(dat['physics']);ctrl=dat['controls']
fit=json.loads((D/'brake-fit.json').read_text());tag='random-5557202'
metrics=csvrows(BASE/'ros'/tag/(tag+'-1.csv'))
pre=json.loads((D/'brake-precontact-fit.json').read_text());fit[:len(pre)]=pre
start=next(i for i,r in enumerate(ctrl) if r['mode']==3)
base=ctrl[start]['scan'];end=ctrl[next(i for i,r in enumerate(metrics) if r['collision']=='1')]['scan']
env,car=make_env(dat['boxes']);rng=np.random.default_rng(12345);rng.normal(0.,.01,size=1080*base)
out=[]
for k in range(base,end+4):
    _,x,y,h=frames[k];car.state[[0,1,4]]=[x,y,h];v=fit[k]['z'][3]
    scan=car.scan_simulator.scan(np.array([x+.27*np.cos(h),y+.27*np.sin(h),h]),rng)
    scan=car.ray_cast_static(scan);hit,b=check_ttc_jit(scan,v,car.scan_angles,car.cosines,car.side_distances,car.ttc_thresh)
    proj=v*car.cosines; mask=(proj>0)&car.static_beams
    clearance=np.where(mask,(scan-car.side_distances)/np.maximum(car.cosines,1.e-6),np.inf)
    closest=int(np.argmin(clearance))
    out.append(dict(frame=k,k=k-base,time=frames[k,0]-frames[base,0],v=v,hit=bool(hit),beam=int(b),
                    forward_envelope_gap_m=float(clearance[closest]),closest_beam=closest,
                    hit_kind=("obstacle" if car.static_beams[b] else "wall") if hit else None,
                    hit_range=float(scan[b]) if hit else None,side_distance=float(car.side_distances[b]) if hit else None))
(D/'recorded-brake-beams.json').write_text(json.dumps(out,indent=2))
print('ROS collision present by',end-base, 'first beam',next((r for r in out if r['hit']),None))
print('brake frame',out[0], 'last',out[-1]);print('pose still at',[(e['k'],e['v']) for e in out if np.linalg.norm(frames[e['frame'],1:3]-frames[e['frame']-1,1:3])<1e-7][:2])
