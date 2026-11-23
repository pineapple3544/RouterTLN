from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import subprocess,json
d=Path(__file__).resolve().parent
exe=d/'plan-eval'
layouts=json.loads((d/'layouts.json').read_text())
def run(c,tag,cpu):
 f=d/(tag+'.json');f.write_text(json.dumps(dict(device='cpu',teacher=False,cases=[c])))
 with (d/(tag+'.log')).open('w') as log:
  subprocess.run(['taskset','-c',str(cpu),str(exe),'models/avoid-corridor-v2.pt',str(d/(tag+'.jsonl')),str(f),'models/race-berlin-wsp.pt'],check=True,stdout=log,stderr=log)
 z=json.loads((d/(tag+'.jsonl')).read_text());print(tag,z['reason'],z['teacher_steps'],flush=True)
def experiment(arg):
 key,label,at,was_planned,cpu=arg
 c=dict(layouts[key],speed_cap=5.25,straight_boost=False,race_sight_speed=not was_planned,race_exit_speed=not was_planned,race_lap_speed=was_planned)
 snap=str(d/(label+'.snapshot'))
 run(dict(c,checkpoint=dict(at_m=at,save=snap)),label+'-save',cpu)
 for idx,(yaw,v) in enumerate([(0,0),(-.02,-.1),(.02,.1)]):
  for who in ['tln','teacher'] if idx==0 or label=='corner' else ['tln']:
   profile=dict(race_lap_speed=True,race_exit_speed=False,race_sight_speed=False,speed_cap=5.25)
   tag=label+'-'+who+'-'+str(idx)
   cp=dict(restore=snap,horizon_ticks=75,perturb=dict(yaw_rad=yaw,speed_mps=v),race_branch=profile,compare_teacher_steering=(who=='teacher'),ticks=str(d/(tag+'.ticks')))
   run(dict(c,checkpoint=cp),tag,cpu)
with ThreadPoolExecutor(max_workers=4) as pool:
 list(pool.map(experiment,[('new-1-051','corner',49.4,False,4),('new-3-028','three-a',49.4,False,5),('s49140402-n3','three-b',147.3,False,10),('new-1-051','later',258.2,True,11)]))
