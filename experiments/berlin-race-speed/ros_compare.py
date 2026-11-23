"""Matched 180-second/10-lap races using simulator truth only for scoring."""
import argparse
import csv
import json
import os
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from score_drive import route_progress
FROZEN=json.loads((HERE/'ros-layouts.json').read_text())
ROUTE=json.loads((ROOT/'maps/berlin/global_waypoints.json').read_text())['global_traj_wpnts_iqp']['wpnts']

def score(tag):
    def read(suffix):
        with Path(str(tag)+suffix).open() as stream:return list(csv.DictReader(stream))
    controls=read('.csv')
    raw=read('-judge.csv')
    first=float(controls[0]['stamp'])
    start=max(0,next(i for i,r in enumerate(raw) if float(r['stamp'])>=first)-1)
    samples=[r for r in raw[start:] if float(r['stamp'])<=first+180]
    time=np.array([float(r['stamp']) for r in samples])
    progress,length=route_progress(samples,ROUTE)
    crossings=[]
    for i,(x,y) in enumerate(zip(progress,progress[1:])):
        goal=length*(len(crossings)+1)
        if x<goal<=y:
            crossings.append(time[i]+(time[i+1]-time[i])*(goal-x)/(y-x))
            if len(crossings)==10:break
    end=crossings[-1] if len(crossings)==10 else time[-1]
    reports=json.loads(Path(str(tag)+'-score.json').read_text())
    hits=[t for t,r in zip(time,samples) if t<=end and r['collision']!='none']
    intervals=np.diff([first,*crossings]).tolist()
    laps_before_hit=sum(t<(hits[0] if hits else float('inf')) for t in crossings)
    moving=False
    stopped=False
    for t,r in zip(time,samples):
        if t<first or t>=end:continue
        v=abs(float(r['speed']))
        moving |= v>.1
        if moving and v<.05:stopped=True
    flying=[abs(float(r['speed'])) for t,r in zip(time,samples) if crossings and crossings[0]<t<=end]
    faults=[r['reason'] for r in controls if r['mode']=='3' and float(r['stamp'])<=end]
    return dict(laps=len(crossings),consecutive_clean=int(laps_before_hit),lap_s=intervals,
                best_s=min(intervals,default=None),mean_flying_s=float(np.mean(intervals[1:])) if len(intervals)>1 else None,
                first_lap_s=intervals[0] if intervals else None,peak_mps=max(flying,default=None),
                p95_mps=float(np.percentile(flying,95)) if flying else None,
                collision=bool(hits),stopped=stopped,emergency=faults[0] if faults else reports.get('controller_fault'),
                elapsed_s=end-first,judge_samples=len(samples),first_plan_ready=controls[0]['plan_ready'])

def trial(job,lane,lo,hi,folder):
    c,variant=job; on=variant=='adaptive'
    tag=folder/(str(c['count'])+'-'+variant)
    sim,controller=[('0,1','2,3'),('4,5','6,7')][lane]
    env=dict(os.environ,ROS_DOMAIN_ID=str(121+lane),EL_SIM_CPUSET=sim,EL_CONTROL_CPUSET=controller,
             EL_MAP_DIR='/workspace/maps/berlin',EL_START_INDEX=str(c['start']),EL_STOP_AFTER_LAPS='10',
             EL_OBSTACLE_INDEX=str(c['index']),EL_OBSTACLE_STRIDE=str(c['stride']),EL_OBSTACLE_OFFSETS=json.dumps(c['offsets']),
             EL_SCAN_NOISE_SEED=str(c['seed']),EL_LAP_PLANNING='true',EL_INFERENCE_DEVICE='cpu',
             EL_CONTROLLER_BINARY=os.environ.get('EL_CONTROLLER_BINARY','/build/el_controller'),EL_SIM_SECTION_SPEED_PROFILE='true' if on else 'false',
             EL_SPEED_CAP=str(hi if on else 3.0),EL_SIM_AVOID_MIN='2.7',EL_SIM_AVOID_MAX='3.0',
             EL_SIM_RACE_ONLY='true' if on else 'false',EL_RACE_BRAKE_PREVIEW_S=str(lo),
             EL_AVOID_CHECKPOINT=str(ROOT/'models'/'avoid-corridor-v2.pt'),
             EL_RACE_CHECKPOINT=str(ROOT/'models/race-berlin-wsp.pt'))
    env.pop('EL_UNIFIED_CHECKPOINT',None)
    with Path(str(tag)+'-run.log').open('w') as output:
        r=subprocess.run(['taskset','-c',sim+','+controller,'bash','scripts/smoke.sh','184',str(c['count']),
                'moe',str(tag.relative_to(ROOT/'runs')),'',str(c['seed'])],env=env,cwd=ROOT,stdout=output,stderr=output)
    try:
        result=score(tag)
    except (ValueError,IndexError,OSError,KeyError,StopIteration) as err:
        result={'scoring_error':str(err)}
    result.update(variant=variant,count=c['count'],seed=c['seed'],runner_exit=r.returncode,
                  race_cap_mps=hi if on else 3.0,preview_s=lo if on else 0.)
    Path(str(tag)+'-verdict.json').write_text(json.dumps(result,indent=2)+'\n')
    print(c['count'],variant,result.get('laps'),result.get('best_s'),result.get('emergency'),r.returncode,flush=True)

if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('preview_s',type=float);parser.add_argument('race_cap',type=float)
    parser.add_argument('variants',nargs='+',choices=['base','adaptive'])
    parser.add_argument('--resume',action='store_true')
    args=parser.parse_args()
    folder=ROOT/'runs/berlin-speed-replay'/f'ros-{args.race_cap:g}-{args.preview_s:g}'
    folder.mkdir(parents=True,exist_ok=True)
    jobs=[(c,v) for c in FROZEN for v in args.variants]
    def work(lane):
        for job in jobs[lane::2]:
            if (folder/f"{job[0]['count']}-{job[1]}-verdict.json").exists():
                if args.resume:continue
                raise FileExistsError('existing trial')
            trial(job,lane,args.preview_s,args.race_cap,folder)
    with ThreadPoolExecutor(max_workers=2) as workers:list(workers.map(work,range(2)))
