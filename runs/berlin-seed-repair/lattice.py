"""Replay just the reachability constraints of the production lattice on a shifted grid."""
from diagnose import *
from PIL import Image

def rows(frame,phase,margin=.065,sub=1):
    grid=base.copy()
    grid.flat[frame['obstacle_cells']]=True
    n=frame['waypoint']
    x,y,h=frame['pose']
    w=wp[n]
    d=station(frame)['lateral_m']; yawerr=math.remainder(h-w[2],math.tau)
    step=.10/sub; mid=12*sub
    origin=int(np.floor((d-phase)/step+.5))+mid
    ds=np.arange(-mid,mid+1)*step+phase
    last=1<<origin;distance=0; masks=[last]

    def clear(xx,yy,aa):
        cx=xx[:,None]+np.cos(aa)*dx-np.sin(aa)*dy
        cy=yy[:,None]+np.sin(aa)*dx+np.cos(aa)*dy
        return ~grid[np.floor((cy-y0)/res).astype(int),np.floor((cx-x0)/res).astype(int)].any(axis=1)
    for i in range(1,32):
        a=wp[(n+2*i)%len(wp)];prev=wp[(n+2*i-2)%len(wp)]
        distance+=np.linalg.norm(a[:2]-prev[:2])
        ex=a[0]-np.sin(a[2])*ds;ey=a[1]+np.cos(a[2])*ds
        eh=a[2]+yawerr*max(0,1-distance/.6604)
        room=.065+(margin-.065)*min(1.,max(0.,distance-.65)/.65)
        dx,dy=np.meshgrid(np.linspace(-(.29+room),.29+room,7),np.linspace(-(.1016+room),.1016+room,5))
        dx,dy=dx.flatten(),dy.flatten()
        free=clear(ex,ey,eh)
        if i==1:
            for t in (.25,.5,.75):free &= clear(x+t*(ex-x),y+t*(ey-y),h+t*math.remainder(eh-h,math.tau))
        allowed=sum(1<<int(j) for j in np.flatnonzero(free))
        neighbors=last
        for shift in range(1,sub+1):neighbors |= (last<<shift) | (last>>shift)
        last=neighbors&allowed
        masks.append(last)
        if not last:return i
    return -1

if __name__=='__main__':
    for seed in ['39103','39104']:
        frame=json.loads((HERE/'cross'/seed/'ff/frame.json').read_text())
        print(seed,[(phase,rows(frame,phase)) for phase in [-.05,-.04,-.03,-.02,-.01,0,.01,.02,.03,.04,.05]])
