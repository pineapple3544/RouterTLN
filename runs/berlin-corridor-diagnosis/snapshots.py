"""Export exact full live occupancy (static map + observed non-wall cells) in ROS map format."""
from pathlib import Path
import json
import math
import os
import subprocess
import yaml
from PIL import Image
import numpy as np
from compare import analyze,base,meta,wp

root=Path(__file__).resolve().parent
def export(folder,frame):
    folder.mkdir(parents=True,exist_ok=True)
    grid=base.copy()
    grid.flat[frame['obstacle_cells']]=True
    Image.fromarray(np.where(grid[::-1],0,255).astype('uint8')).save(folder/'map.png')
    (folder/'map.yaml').write_text(yaml.safe_dump(dict(meta,image='map.png')))
    (folder/'frame.json').write_text(json.dumps(frame)+'\n')
def paired():
    result={}
    for layout in ['original','distributed']:
        def read(n):
            f=root/'before'/f'{layout}-{n}.csv.corridor.jsonl'
            return f,[json.loads(line) for line in f.open()]
        f,a=read(1);g,b=read(2)
        i=next(i for i,r in enumerate(a) if r['first_empty']>0)
        bad=a[i];good=min(b,key=lambda r:abs(r['progress']-bad['progress']))
        assert bad['waypoint']==good['waypoint']
        frames={}
        for label,row in [('previous',a[i-1]),('failure',bad),('complete',good)]:
            export(root/'snapshots'/layout/label,row)
            record=analyze(f if label!='complete' else g,row)
            x,y,h=wp[row['waypoint']]
            record['lateral_m']=float(-(row['pose'][0]-x)*math.sin(h)+(row['pose'][1]-y)*math.cos(h))
            record['heading_error_rad']=math.remainder(row['pose'][2]-h,2*math.pi)
            frames[label]=record
        export(Path('tests/fixtures/corridor')/layout,bad)
        swaps={}
        for name,pose in [('failed',bad),('completed',good)]:
            for grid_name,grid in [('failed',bad),('completed',good)]:
                mixed=dict(pose,obstacle_cells=grid['obstacle_cells'])
                swaps[name+'_pose_'+grid_name+'_grid']=analyze(f,mixed)['dead'].get('full',-1)
        result[layout]={'observations':frames,'crossed_first_empty':swaps}
    (root/'diagnosis.json').write_text(json.dumps(result,indent=2)+'\n')

if __name__=='__main__':paired()
