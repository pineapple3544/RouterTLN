import numpy as np,json,yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt
from scipy.spatial import cKDTree
from pathlib import Path
d=Path('runs/berlin-scout-speed-plan')
m=Path('maps/berlin');cfg=yaml.safe_load((m/'berlin.yaml').read_text());res=cfg['resolution'];ox,oy=cfg['origin'][:2];im=np.asarray(Image.open(m/cfg['image']).convert('L'))[::-1];dist=distance_transform_edt(im/255.0>1-cfg['free_thresh'])*res
wp=json.loads((m/'global_waypoints.json').read_text())['global_traj_wpnts_iqp']['wpnts'];pts=np.array([[v['x_m'],v['y_m']] for v in wp]);yaw=np.array([v['psi_rad'] for v in wp]);tree=cKDTree(pts)
xx,yy=np.meshgrid(np.linspace(-.29,.29,9),np.linspace(-.1016,.1016,5))
def sample(name):
 a=[json.loads(l) for l in (d/(name+'.ticks')).open()];a=[t for t in a if t['mode']!=3]
 margins=[];lats=[]
 for t in a:
  x,y,h,v,_=t['car'];c,s=np.cos(h),np.sin(h);X=x+c*xx-s*yy;Y=y+s*xx+c*yy
  margins.append(dist[((Y-oy)/res).astype(int),((X-ox)/res).astype(int)].min())
  _,idx=tree.query([x,y]);lats.append(abs(-(x-pts[idx,0])*np.sin(yaw[idx])+(y-pts[idx,1])*np.cos(yaw[idx])))
 return (round(min(margins),2),round(max(lats),2),round(a[-1]['car'][3],2)) if a else None
for name in ['corner','three-a','three-b','later']:
 print(name,[(actor,sample(name+'-'+actor+'-0')) for actor in ['tln','teacher']])
