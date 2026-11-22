"""Audit finished Berlin trials against the original ROS layout checks and speed band."""
import csv
import json
import runpy
import sys
from pathlib import Path
from statistics import mean

here = Path(__file__).resolve().parent
analyze = runpy.run_path(str(here.parent / 'berlin-layout-sweep/analyze.py'))['analyze']

def summarize(directory):
    trials = analyze(directory)
    points = json.loads((here.parents[1] / 'maps/berlin/global_waypoints.json').read_text())
    length = points['global_traj_wpnts_iqp']['wpnts'][-1]['s_m']
    for trial in trials:
        with (directory / (trial['tag'] + '.csv')).open() as stream:
            rows = list(csv.DictReader(stream))
        flying = [row for row in rows if row['mode'] == '1' and row['plan_ready'] == '1' and row['valid'] == '1']
        commands = [float(row['target_speed']) for row in flying]
        speeds = [float(row['speed']) for row in flying]
        trial['flying_avoid_samples'] = len(flying)
        trial['avoid_samples_per_flying_lap'] = [
            sum(n*length - .01 <= float(row['progress_m']) < (n+1)*length - .01 for row in flying)
            for n in (1, 2)
        ]
        trial['command_range'] = [min(commands), max(commands)] if commands else []
        trial['actual_range'] = [min(speeds), max(speeds)] if speeds else []
        trial['actual_mean'] = round(mean(speeds), 3) if speeds else None
        trial['first_bad_tick_ms'] = next((float(row['compute_ms']) for row in rows if row['mode'] == '3'), None)
        trial['accepted'] = bool(trial['finished'] and trial['plan_ready_m'] is not None
                                 and len(trial['brakes_by_lap']) == 3
                                 and all(count > 0 for count in trial['brakes_by_lap'][1:]) and commands
                                 and all(trial['avoid_samples_per_flying_lap'])
                                 and min(commands) >= 2.7 - 1e-5 and max(commands) <= 3.0 + 1e-5)
    return trials

if __name__ == '__main__':
    print(json.dumps(summarize(Path(sys.argv[1])), indent=2))
