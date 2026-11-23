"""Paired speed trials. Use the same LiDAR-only controller and restart every case."""
import json
import os
import statistics
import subprocess
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
BINARY=Path(os.getenv("EL_EVAL_BINARY",'/build/el_eval_avoid'))
CORES=list(map(int,os.getenv('EVAL_CORES','0,2,4,6,8,10,12,14').split(',')))
def results(folder):
    return {z['spec']['name']:z for f in folder.glob('results-*.jsonl') for z in map(json.loads,f.open())}
def passed(z):
    return (z['pose_source']=='lidar_static_map' and z['reason']=='completed' and len(z['laps'])==3
            and not z['contact'] and not z['first_rejection'] and z['min_speed_after_first_half_m']>=.05)
def flying(z):return statistics.mean(l['seconds'] for l in z['laps'][1:3])
def run(label,cases,low,high,adaptive,trace=False):
    folder=Path(os.environ.get('EL_SPEED_OUTPUT',str(ROOT/'runs/berlin-speed-replay')))/label
    folder.mkdir(parents=True)
    def worker(i):
        group=[]
        for c in cases[i::len(CORES)]:
            c=dict(c,sim_avoid_min=low,sim_avoid_max=3.0 if c.get("sim_race_only") else high,speed_cap=high,sim_section_speed_profile=adaptive)
            if trace:c['checkpoint']={'ticks':str(folder/(c['name']+'.ticks'))}
            group.append(c)
        if not group:return
        cfg=folder/f'cases-{i}.json'
        cfg.write_text(json.dumps(dict(device='cpu',teacher=False,cases=group)))
        with (folder/f'worker-{i}.log').open('w') as out:
            subprocess.run(['taskset','-c',str(CORES[i]),str(BINARY),os.environ.get('EL_AVOID_CHECKPOINT','models/avoid-corridor-v2.pt'),
                str(folder/f'results-{i}.jsonl'),str(cfg),'models/race-berlin-wsp.pt'],check=True,cwd=ROOT,stdout=out,stderr=out)
    with ThreadPoolExecutor(max_workers=len(CORES)) as pool:list(pool.map(worker,range(len(CORES))))
    data=results(folder)
    assert set(data)=={c['name'] for c in cases}
    return data
def compare(data,base):
    assert set(data)==set(base)
    ok={k for k,v in data.items() if passed(v)}
    prior={k for k,v in base.items() if passed(v)}
    both=sorted(prior&ok)
    scout=[abs(base[k]['laps'][0]['seconds']-z['laps'][0]['seconds']) for k,z in data.items() if base[k]['laps'] and z['laps']]
    assert all(v['pose_source']=='lidar_static_map' for v in data.values())
    return dict(max_first_lap_delta_s=max(scout,default=0),first_lap_count=sum(bool(v["laps"]) for v in data.values()),trials=len(data),clean=len(ok),base_clean=len(prior),lost=sorted(prior-ok),gained=sorted(ok-prior),
        paired=len(both),base_flying_s=statistics.mean(flying(base[k]) for k in both) if both else None,
        flying_s=statistics.mean(flying(data[k]) for k in both) if both else None,
        collision=sum(z['contact'] is not None for z in data.values()),
        failures={k:(v['first_rejection'] or {}).get('reason',v['reason']) for k,v in data.items() if k not in ok})

if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser(description='Run matched three-lap Berlin layouts with fresh scout plans.')
    parser.add_argument('dataset',choices=['pilot','evaluation'])
    parser.add_argument('cap',type=float)
    parser.add_argument('preview',type=float)
    args=parser.parse_args()
    cases=json.loads((HERE/(args.dataset+'.json')).read_text())
    label=f'{args.dataset}-{args.cap:g}-{args.preview:g}'
    base=run(label+'-base',cases,2.7,3.,False)
    specs=[dict(c,sim_race_only=True,race_brake_preview_s=args.preview) for c in cases]
    candidate=run(label+'-candidate',specs,2.7,args.cap,True)
    print(json.dumps(compare(candidate,base),indent=2))
