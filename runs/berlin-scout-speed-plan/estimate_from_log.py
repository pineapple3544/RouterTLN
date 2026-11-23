from pathlib import Path
import numpy as np, csv,json
from scipy.signal import savgol_filter
p=Path('runs/berlin-12s-implementation/ros-production-repeat')
def read(suffix):
 with Path(str(p)+suffix).open() as f:return list(csv.DictReader(f))
a=read('.csv');b=read('-judge.csv');z=json.loads(Path(str(p)+'-score.json').read_text());laps=z['lap_s'];best=np.argmin(laps);start=float(a[0]['stamp'])+sum(laps[:best]);end=start+laps[best]
times=np.array([float(x['stamp']) for x in b]); grid=np.arange(start,end,.025)
def sample(k):return np.interp(grid,times,[float(x[k]) for x in b])
xy=np.array([sample('x'),sample('y')]).T
ds=np.linalg.norm(np.roll(xy,-1,axis=0)-xy,axis=1); v=sample('speed')
stamps=np.array([float(x['stamp']) for x in a]);mode=np.array([int(x['mode']) for x in a])[np.searchsorted(stamps,grid,side='right')-1]
theta=np.unwrap(np.arctan2(np.gradient(xy[:,1]),np.gradient(xy[:,0])))
k=np.abs(savgol_filter(theta,9,2,deriv=1,delta=.025))/np.maximum(v,.3)
acc=savgol_filter(v,9,2,deriv=1,delta=.025)
for q in [90,95,99]:
 race=mode==0
 lat=np.percentile((v*v*k)[race],q);up=np.percentile(acc[race & (acc>0)],q);down=np.percentile(-acc[race & (acc<0)],q)
 for cap in (5.25,10):
  w=np.minimum(cap**2,lat/np.maximum(k,.001));w[~race]=v[~race]**2
  for _ in range(2):
   for i in reversed(range(len(w))):w[i]=min(w[i],w[(i+1)%len(w)]+2*down*ds[i])
   for i in range(len(w)):
    j=(i+1)%len(w); w[j]=min(w[j],w[i]+2*up*ds[i])
  dt=ds*2/(np.sqrt(w)+np.sqrt(np.roll(w,-1)))
  print(q,cap,'lap',round(dt[race].sum()+.025*(~race).sum(),3),'lat/up/down',np.round([lat,up,down],2),'length',round(ds.sum(),2))
