from pathlib import Path
import json
import math
import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon,Patch
from matplotlib.lines import Line2D
from compare import base,wp,x0,y0,res,W,H

p=Path(__file__).resolve().parent
fig,axes=plt.subplots(2,2,figsize=(10,7.5),layout='constrained')
def rectangle(x,y,angle,length,width):
    corners=np.array([[-length,-width],[length,-width],[length,width],[-length,width]])*.5
    c,s=math.cos(angle),math.sin(angle)
    return corners@np.array([[c,s],[-s,c]])+[x,y]
for row,layout in enumerate(['original','distributed']):
    poses=next(yaml.safe_load_all((p/'before'/(layout+'-1-poses.yaml')).read_text()))['poses']
    for ax,kind in zip(axes[row],['failure','complete']):
        r=json.loads((p/'snapshots'/layout/kind/'frame.json').read_text())
        x,y,h=r['pose'];wx,wy,_=wp[r['waypoint']]
        ax.imshow(base,origin='lower',extent=(x0,x0+res*W,y0,y0+res*H),cmap='Greys',vmin=0,vmax=1.35,interpolation='nearest')
        cells=np.array(r['obstacle_cells'])
        for k in cells:
            xx=x0+(k%W+.5)*res;yy=y0+(k//W+.5)*res
            if abs(xx-wx)<1.8 and abs(yy-wy)<1.2:
                ax.add_patch(Polygon(rectangle(xx,yy,0,res,res),facecolor='#ef9d32',edgecolor='none'))
        for z in poses:
            ax.add_patch(Polygon(rectangle(z['position']['x'],z['position']['y'],2*math.atan2(z['orientation']['z'],z['orientation']['w']),.3,.3),fill=False,edgecolor='#a65012',lw=1.8))
        ax.add_patch(Polygon(rectangle(x,y,h,.58,.2032),facecolor='#68a2ca',alpha=.65,edgecolor='#164b77'))
        ax.arrow(x,y,.33*math.cos(h),.33*math.sin(h),head_width=.055,color='#164b77',length_includes_head=True)
        cx,cy,yaw=wp[(r['waypoint']+2)%len(wp)]
        for j in range(max(0,r['origin']-1),min(25,r['origin']+2)):
            d=(j-12)*.1
            ok=r['reachable'][1]&(1<<j)
            ax.plot(cx-math.sin(yaw)*d,cy+math.cos(yaw)*d,'o' if ok else 'x',color='#098655' if ok else '#b72134',ms=7,mew=2)
        ax.set(xlim=(wx-1.4,wx+1.4),ylim=(wy-.95,wy+.95),aspect='equal',xlabel='Map x (m)',ylabel='Map y (m)')
        ax.set_title(f'{layout.capitalize()}  |  {kind}  |  s = {r["progress"]:.2f} m')
fig.legend(handles=[Patch(color='#68a2ca',label='Vehicle'),Patch(color='#ef9d32',label='Observed cells'),Patch(fill=False,edgecolor='#a65012',label='Actual obstacle'),Line2D([],[],marker='x',ls='',color='#b72134',label='Blocked first step'),Line2D([],[],marker='o',ls='',color='#098655',label='Reachable first step')],loc='upper center',bbox_to_anchor=(.5,1.065),ncol=5)
fig.savefig(p/'entry-comparison.png',bbox_inches='tight',dpi=165)
