"""Compare the observed brake path with immediate and earlier braking using the same Gym."""
from geometry import *
from render import make_env, install
from f110_gym.envs.laser_models import check_ttc_jit
def run():
    raw=json.loads((D/'brake.json').read_text());ctrl=raw['controls'];ph=np.array(raw['physics']);fit=json.loads((D/'brake-fit.json').read_text())
    pre=json.loads((D/'brake-precontact-fit.json').read_text());fit[:len(pre)]=pre
    first=next(i for i,r in enumerate(ctrl) if r['mode']==3)
    base=ctrl[first]['scan']; begin=base-10; boxes=raw['boxes']
    env,car=make_env(boxes)
    env.reset(poses=np.array([[*ph[begin,1:]]]))
    runs=[]
    for shift in [None,0,-1,-2,-3,-5,-8]:
        st=fit[begin]['z'][:];st[0],st[1],st[4]=ph[begin,1:]
        install(env,st,fit[begin]['delay'],begin,0)
        points=[(st[0],st[1])]; trace=[];contact=None
        for k in range(begin+1,base+100):
            ci=fit[k]['command'];u=ctrl[ci]
            if shift is not None and k>=base+1+shift:
                u=dict(steer=ctrl[first]['steer'],target=0.)
            # Run the same vehicle and LiDAR update as Gym. Keep rolling after a
            # predicted contact to measure the unobstructed geometric stop distance.
            scan=car.update_pose(u['steer'],u['target']);scan=car.ray_cast_static(scan)
            v=float(car.state[3])
            hit,beam=check_ttc_jit(scan,v,car.scan_angles,car.cosines,car.side_distances,car.ttc_thresh)
            x,y,h=car.state[[0,1,4]];points.append((x,y))
            gap=min(sat(x,y,h,b) for b in boxes)
            trace.append(dict(frame=k,step_from_brake=k-base,v=v,gap_m=gap,wall_m=wall(x,y,h)))
            if hit and contact is None:
                contact=dict(frame=k,from_brake=k-base,kind='obstacle' if car.static_beams[beam] else 'wall',beam=int(beam),
                             laser_m=scan[beam],ttc_envelope_m=car.side_distances[beam],v=v,gap_m=gap)
            if k>base and v<.005:break
        points=np.array(points)
        stopdist=np.linalg.norm(np.diff(points[base-begin:],axis=0),axis=1).sum()
        runs.append(dict(brake=shift,contact=contact,travel_after_decision_m=stopdist,min_box_m=min(t['gap_m'] for t in trace),
                         min_wall_m=min(t['wall_m'] for t in trace),trace=trace))
    (D/'brake-variants.json').write_text(json.dumps(runs,indent=2))
    for r in runs:print({k:v for k,v in r.items() if k!='trace'})
if __name__=='__main__':run()
