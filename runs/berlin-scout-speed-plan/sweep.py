from pathlib import Path
import os,subprocess,json,sys
from concurrent.futures import ThreadPoolExecutor
d=Path(__file__).resolve().parent
layouts=json.loads((d/'layouts.json').read_text())
label,mode,speeds,group=sys.argv[1:5]
cases=[]
for key,base in layouts.items():
 if group!='six' and key!=group:continue
 for speed in map(float,speeds.split(',')):
  c=dict(base,name=f'{key}-{speed:g}',speed_cap=speed,straight_boost=False)
  if mode=='planned':c.update(race_lap_speed=True)
  elif mode!='reference':c.update(race_sight_speed=bool(int(mode)&1),race_exit_speed=bool(int(mode)&2))
  cases.append(c)
folder=d/label;folder.mkdir()
def trial(pair):
 i,c=pair;tag=folder/c['name']
 if label=='ref':
  c['checkpoint']={'ticks':str(tag)+'.ticks','save':str(tag)+'.snapshot','at_m':48.5}
 path=Path(str(tag)+'.json')
 path.write_text(json.dumps(dict(cases=[c],teacher=False,device='cpu')))
 exe=os.getenv('EL_EVAL_BINARY','/build/el_eval_avoid')
 with Path(str(tag)+'.log').open('w') as f:
  subprocess.run(['taskset','-c',str(i%12),exe,'models/avoid-corridor-v2.pt',str(tag)+'.jsonl',str(path),'models/race-berlin-wsp.pt'],check=True,stdout=f,stderr=f)
 z=json.loads(Path(str(tag)+'.jsonl').read_text())
 print(c['name'],z['reason'],len(z['laps']),min([l['seconds'] for l in z['laps']] or [0]),flush=True)
with ThreadPoolExecutor(max_workers=12) as pool:list(pool.map(trial,enumerate(cases)))
