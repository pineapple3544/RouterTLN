"""Plot sensed occupancy and matched entry trajectories in the failed vehicle frame."""
from pathlib import Path
import json, sys, math
import numpy as np
from PIL import Image
import yaml
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

D=Path(__file__).resolve().parent
M=D.parents[1]/'maps/berlin'
meta=yaml.safe_load((M/'berlin.yaml').read_text())
walls=np.array(Image.open(M/meta['image']).convert('L'))[::-1]<205
wp=json.loads((M/'global_waypoints.json').read_text())['global_traj_wpnts_iqp']['wpnts']
root=D/sys.argv[1]
fig,axes=plt.subplots(2,3,figsize=(15,7),layout='constrained')
for ax,f in zip(axes.flat,sorted((D/'before').glob('*.ticks'))):
    rr=[json.loads(l) for l in f.open()]
    fail=next(r for r in rr if r['reason']=='no_corridor')
    updated=[json.loads(l) for l in (root/f.name).open()]
    yaw=wp[fail['waypoint']]['psi_rad'];c,s=np.cos(yaw),np.sin(yaw)
    frame=np.array([[c,-s],[s,c]])
    origin=np.array(fail['car'][:2])
    def local(xy):return (np.array(xy)-origin)@frame
    xx,yy=np.meshgrid(np.linspace(-2.7,2,320),np.linspace(-1.5,1.5,200))
    xy=np.stack([xx,yy],axis=-1)@frame.T+origin
    pix=np.floor((xy-meta['origin'][:2])/meta['resolution']).astype(int)
    ax.imshow(np.where(walls[pix[:,:,1],pix[:,:,0]],.74,.985),origin='lower',vmin=0,vmax=1,cmap='gray',extent=[-2.7,2,-1.5,1.5])
    ids=np.array(fail['occupancy']);pts=np.stack([ids%walls.shape[1],ids//walls.shape[1]],axis=-1)
    obs=local((pts+.5)*meta['resolution']+np.array(meta['origin'][:2]))
    ax.scatter(*obs.T,s=7,marker='s',c='#374251')
    e=fail['first_empty']
    for i,key,color in [(e,'free_columns','#5dbbb5'),(e-1,'reachable_columns','#6334ad')]:
        row=fail[key][i];cols=[j for j in range(49) if row[j//32]>>(j%32)&1]
        w=wp[(fail['waypoint']+i*2)%(len(wp)-1)]
        pts=local(np.array([w['x_m'],w['y_m']])+((np.array(cols)-24)*.05)[:,None]*np.array([-math.sin(w['psi_rad']),math.cos(w['psi_rad'])]))
        ax.scatter(*pts.T,c=color,s=12,zorder=4)
    for series,col,ls in [(rr,'#bb4c32','-'),(updated,'#136198','--')]:
        pts=[z['car'][:2] for z in series if fail['s']-3<z['s']<fail['s']+2 and z['mode']!=3]
        if pts:ax.plot(*local(pts).T,c=col,ls=ls,lw=2.1)
    ax.plot(0,0,'x',c='black',ms=8)
    ax.set(xlim=(-2.7,2),ylim=(-1.5,1.5),aspect='equal',title=f"{f.stem[7:]}   |   {fail['s']:.2f} m   |   stage {e}",xlabel='Along track (m)',ylabel='Left (m)')
axes.flat[-1].axis('off')
axes.flat[-1].legend(handles=[Line2D([],[],c='#bb4c32',label='95-pass build'),Line2D([],[],c='#136198',ls='--',label='Patched build'),Patch(color='.74',label='Mapped wall'),Patch(color='#374251',label='Observed occupancy'),Patch(color='#6334ad',label='Reachable in preceding row'),Patch(color='#5dbbb5',label='Free in disconnected row'),Line2D([],[],marker='x',color='black',ls='',label='First refusal')],loc='center left',frameon=False)
fig.savefig(D/'entry-grids.png',dpi=175)
