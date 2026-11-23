"""Summarize paired closed-loop trials and actual fresh scan delivery during control."""
import csv
import hashlib
import json
import runpy
import subprocess
from pathlib import Path
import numpy as np

root=Path(__file__).resolve().parent
summarize=runpy.run_path(str(root.parent/'berlin-brake-merge/analyze.py'))['summarize']
def sensor_intervals(folder):
    total=duplicates=overlaps=bad=0
    header_max=arrival_max=0.
    for f in sorted(folder.glob('*.csv.receive.csv')):
        metrics=f.with_name(f.name.replace('.receive.csv',''))
        with metrics.open() as stream:rows=list(csv.DictReader(stream))
        lo,hi=float(rows[0]['stamp']),float(rows[-1]['stamp'])
        with f.open() as stream:
            samples=np.array([[float(rx),float(ts)] for rx,ts in csv.reader(stream) if lo<=float(ts)<=hi])
        gaps=np.diff(samples,axis=0)
        total+=len(gaps); duplicates+=int((gaps[:,1]==0).sum())
        header_max=max(header_max,float(gaps[:,1].max())*1000)
        arrival_max=max(arrival_max,float(gaps[:,0].max())*1000)
        physics=[]
        bridge=f.with_name(f.name.replace('.csv.receive.csv','-bridge.csv'))
        for line in bridge.read_bytes().splitlines()[1:]:
            try:
                ev,start,end,stamp=line.decode().split(',')
                start,end,stamp=map(float,(start,end,stamp))
                if ev not in ('physics','publish') or not 0<=end-start<=1:raise ValueError()
            except (UnicodeDecodeError,ValueError):
                bad+=1
                continue
            if ev=='physics' and lo<=stamp<=hi:physics.append((start,end))
        physics.sort()
        overlaps+=sum(end>following for (_,end),(following,_) in zip(physics,physics[1:]))
    return dict(received_intervals=total,duplicates=duplicates,physics_overlaps=overlaps,
                excluded_bridge_log_lines=bad,max_header_gap_ms=header_max,max_arrival_gap_ms=arrival_max)

if __name__=='__main__':
    report={'conditions':{'race':'race-berlin-wsp.pt','avoid':'avoid-corridor-v2.pt',
        'sha256':hashlib.sha256(Path('models/avoid-corridor-v2.pt').read_bytes()).hexdigest(),
        'laps':3,'flying_avoid_band_mps':[2.7,3.0],'sensor_limit_ms':150,'deadline_ms':25},
        'grid_replay':json.loads((root/'diagnosis.json').read_text()),
        'watchdog':json.loads((root/'watchdog.json').read_text()),'runs':{}}
    for name in ['before','after','verify']:
        folder=root/name;trials=summarize(folder)
        assert len(trials)==4
        for r in trials:
            assert 'TLN inference device: cpu' in (folder/(r['tag']+'-controller.log')).read_text()
        result={'trials':trials,'accepted':sum(r['accepted'] for r in trials),
                'collisions':sum(r['collision'] for r in trials)}
        if name!='verify':result['sensor']=sensor_intervals(folder)
        report['runs'][name]=result
    for layout in ['original','distributed']:
        snap=root/'snapshots'/layout/'failure'
        replay=json.loads(subprocess.check_output(['/build/el_replay_corridor',str(snap/'map.yaml'),
            'maps/berlin/global_waypoints.json',str(snap/'frame.json')]))
        report['grid_replay'][layout]['fixed_replay']=replay
    (root/'results.json').write_text(json.dumps(report,indent=2)+'\n')
