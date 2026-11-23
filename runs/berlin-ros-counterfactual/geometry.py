from prepare import *
from math import cos,sin,pi
from PIL import Image
import runpy
sat=runpy.run_path(str(BASE/'audit.py'))['sat_gap']
meta=yaml.safe_load((ROOT/'maps/berlin/berlin.yaml').read_text())
image=np.asarray(Image.open(ROOT/'maps/berlin/berlin.png').convert('L'))[::-1]
dy,dx=np.where(image<=128); res=meta['resolution']
walls=np.column_stack([meta['origin'][0]+(dx+.5)*res,meta['origin'][1]+(dy+.5)*res])
def wall(x,y,yaw):
    p=walls[(abs(walls[:,0]-x)<2.0)&(abs(walls[:,1]-y)<2.0)]-[x,y]
    a,b=cos(yaw),sin(yaw)
    # SAT gap of each occupied cell and the oriented car.
    extent=res/2*(abs(a)+abs(b))
    gaps=np.maximum.reduce([abs(p[:,0]*a+p[:,1]*b)-.29-extent,abs(-p[:,0]*b+p[:,1]*a)-.1016-extent,
                            abs(p[:,0])-.29*abs(a)-.1016*abs(b)-res/2,
                            abs(p[:,1])-.29*abs(b)-.1016*abs(a)-res/2])
    return min(gaps)
if __name__=='__main__':
    dat=json.loads((D/'brake.json').read_text());frames=np.array(dat['physics']);ctrl=dat['controls'];fit=json.loads((D/'brake-fit.json').read_text())
    pre=json.loads((D/'brake-precontact-fit.json').read_text());fit[:len(pre)]=pre
    tag='random-5557202'
    ms=csvrows(BASE/'ros'/tag/(tag+'-1.csv'));first=next(i for i,c in enumerate(ctrl) if c['mode']==3)
    hit=next(i for i,c in enumerate(ms) if c['collision']=='1')
    ci=ctrl[first]['odom'];hc=ctrl[hit]['odom'];path=frames[ci:hc+1];length=np.r_[0,np.cumsum(np.linalg.norm(np.diff(path[:,1:3],axis=0),axis=1))]
    events=[]
    for n,(stamp,x,y,yaw) in enumerate(path):
        events.append(dict(t=stamp-path[0,0],travel_m=length[n],v=fit[ci+n]['z'][3],wall_sat_m=wall(x,y,yaw),box_sat_m=min(sat(x,y,yaw,box) for box in dat['boxes']),applied_index=fit[ci+n]['command']))
    (D/'braking-geometry.json').write_text(json.dumps(events,indent=2))
    print('cmd',ctrl[first],ms[hit]['progress_m'])
    print('total',length[-1], 'min wall',min(e['wall_sat_m'] for e in events))
    print('geometry',events[0],events[-1])
    print('first applied',next((e for e in events if e['applied_index']>=first),None))
    print('overlap',next((e for e in events if e['wall_sat_m']<=0),None))
