"""Validate ROS CSV and scenario publications for each completed trial."""
from pathlib import Path
import csv
import json
import math
import sys
import yaml

ROOT = Path('/workspace')
def analyze(directory):
    w=json.loads((ROOT/'maps/berlin/global_waypoints.json').read_text())['global_traj_wpnts_iqp']['wpnts']
    length=w[-1]['s_m']
    summary=[]
    for layout,(first,stride,offsets) in json.loads((directory/'layouts.json').read_text()).items():
        for repeat in (1,2):
            tag=f'{layout}-{repeat}'
            base=directory/tag
            poses_file=directory/(tag+'-poses.yaml')
            log=directory/(tag+'-run.log')
            if not log.exists() or '"iterations"' not in log.read_text():continue
            msg=next(yaml.safe_load_all(poses_file.read_text()))
            if msg['header']['frame_id']!='map' or len(msg['poses'])!=3:raise ValueError(tag+' poses')
            for i,(pose,d) in enumerate(zip(msg['poses'],offsets)):
                wp=w[first+stride*i]
                yaw=wp['psi_rad']
                x=wp['x_m']-math.sin(yaw)*d
                y=wp['y_m']+math.cos(yaw)*d
                q=pose['orientation']
                if math.hypot(x-pose['position']['x'],y-pose['position']['y'])>1e-4 or abs(abs(q['z']*math.sin(yaw/2)+q['w']*math.cos(yaw/2))-1)>1e-5:
                    raise ValueError(tag+' unexpected obstacle pose')
            with base.with_suffix('.csv').open() as f: rows=list(csv.DictReader(f))
            r0=rows[0]
            assert [int(r0[x]) for x in ('plan_ready','planned_entries','plan_brake_steps')]==[0,0,0],tag
            assert abs(float(r0['progress_m']))<.01 and math.hypot(float(r0['x'])-w[37]['x_m'],float(r0['y'])-w[37]['y_m'])<.15,tag+' initial pose'
            length_eps=.01
            def lap(r):return min(3,1+int((float(r['progress_m'])+length_eps)/length))
            times=[]
            t=float(r0['stamp'])
            for n in range(1,4):
                cross=next((r for r in rows if float(r['progress_m'])>=n*length-length_eps),None)
                if not cross:break
                nt=float(cross['stamp'])
                times.append(round(nt-t,3));t=nt
            end=rows[-1]
            finished=(len(times)==3 and end['mode']=='0' and end['collision']=='0' and
                      end['valid']=='0' and float(end['target_speed'])==0)
            events=[]
            for r in rows:
                if r['reason'] in ('start','return','planned_start'):
                    events.append([lap(r),r['reason'],round(float(r['progress_m']),3)])
            problems=[]
            for r in rows[:-1] if finished else rows:
                if r['collision']!='0' or r['mode']=='3' or r['valid']!='1' or float(r['target_speed'])<=0 or (float(r['progress_m'])>.5 and float(r['speed'])<.05):
                    problems.append(r)
            ready=next((r for r in rows if r['plan_ready']=='1'),None)
            assert not ready or float(ready['progress_m'])>=length-.15,tag+' premature plan'
            failure=problems[0] if problems else None
            previous_brakes=0
            brakes_by_lap=[]
            for n in (1,2,3):
                lap_rows=[r for r in rows if lap(r)==n]
                if lap_rows:
                    brakes=int(lap_rows[-1]['plan_brake_steps'])
                    brakes_by_lap.append(brakes-previous_brakes)
                    previous_brakes=brakes
            summary.append(dict(tag=tag,brakes_by_lap=brakes_by_lap,finished=finished and not problems,lap_times=times,
                plan_ready_m=round(float(ready['progress_m']),3) if ready else None,
                planned_entries=int(end['planned_entries']),brake_steps=int(end['plan_brake_steps']),
                events=events,collision=any(r['collision']!='0' for r in rows),
                stopped=any(float(r['progress_m'])>.5 and float(r['speed'])<.05 for r in rows),
                failure=[lap(failure),round(float(failure['progress_m']),3),failure['reason']] if failure else None,
                end_progress=round(float(end['progress_m']),3)))
    return summary

if __name__=='__main__':
    print(json.dumps(analyze(Path(sys.argv[1])),indent=2))
