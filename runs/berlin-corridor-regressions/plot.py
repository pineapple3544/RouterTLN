"""Map the three simulated trajectories around the original no_corridor pose."""
from pathlib import Path
import json
import math
import numpy as np
from PIL import Image
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt
from matplotlib.patches import Polygon

D=Path(__file__).resolve().parent
M=D.parents[1]/'maps/berlin'
from yaml import safe_load
meta=safe_load((M/'berlin.yaml').read_text());img=np.array(Image.open(M/meta['image']).convert('L'))[::-1]
wp=json.loads((M/'global_waypoints.json').read_text())['global_traj_wpnts_iqp']['wpnts']
fig,axes=plt.subplots(2,2,figsize=(11,7),layout='constrained')
for ax,diagnosis in zip(axes.ravel(),json.loads((D/'failures.json').read_text())):
    tag=diagnosis['layout'];s=diagnosis['first_stop_m']
    traces=[]
    for stage in ('before84','before87','reach-four'):
        traces.append([t for l in (D/stage/(tag+'.ticks')).open() if s-3.5<(t:=json.loads(l))['s']<s+3.2])
    fault=next(t for t in traces[1] if t['reason']=='no_corridor');origin=np.array(fault['car'][:2]);theta=wp[fault['waypoint']]['psi_rad']
    c,h=math.cos(theta),math.sin(theta);axes_matrix=np.array([[c,-h],[h,c]])
    def local(p):return (np.asarray(p)-origin)@axes_matrix
    xx,yy=np.meshgrid(np.linspace(-3.4,3.1,325),np.linspace(-1.6,1.8,170))
    world=np.stack([xx,yy],axis=-1)@axes_matrix.T+origin
    idx=np.floor((world-np.array(meta['origin'][:2]))/meta['resolution']).astype(int)
    bg=np.where(img[idx[:,:,1].clip(0,img.shape[0]-1),idx[:,:,0].clip(0,img.shape[1]-1)]<128,.73,.98)
    ax.imshow(bg,cmap='gray',vmin=0,vmax=1,origin='lower',extent=[-3.4,3.1,-1.6,1.8])
    row=next(z for p in (D/'before87').glob('results*.jsonl') for l in p.open() if (z:=json.loads(l))['case']==tag)
    for x,y,yaw,length,width in row['boxes']:
        bb=np.array([[-1,-1],[-1,1],[1,1],[1,-1]])*[length/2,width/2]
        cc,ss=math.cos(yaw),math.sin(yaw);bb=bb@np.array([[cc,ss],[-ss,cc]])+np.array([x,y])
        ax.add_patch(Polygon(local(bb),color='#332d29'))
    for arr,color,style,label in zip(traces,['#2c75ab','#cc5c2c','#147957'],['--','-',':'],['84-pass build','87-pass build','Fallback search']):
        if label=='87-pass build': arr=arr[:next(i for i,t in enumerate(arr) if t['mode']==3)+1]
        pts=local([t['car'][:2] for t in arr]);ax.plot(*pts.T,color=color,ls=style,lw=2.3,label=label)
    ax.plot(0,0,'x',color='#a1341c',ms=10,mew=2,label='First refusal')
    ax.set_aspect('equal');ax.set_xlim(-3.4,3.1);ax.set_ylim(-1.6,1.8)
    ax.set_title(f"{tag.split('-')[1]}     lap {diagnosis['lap']}  /  row {diagnosis['stage']}")
    ax.set_xlabel('Along local track tangent (m)');ax.set_ylabel('Left of tangent (m)')
axes[0,0].legend(fontsize=8)
fig.savefig(D/'trajectories.png',dpi=155)
