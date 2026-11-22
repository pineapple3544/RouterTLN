"""Audit completed ROS trials and the stages making up the control deadline."""
from pathlib import Path
import csv
import json
import math
import runpy
import sys

STAGES = ('scan','memory','route','observation','corridor','inputs','forward','readback','safety')

def quantiles(values):
    ordered = sorted(values)
    return {key: round(ordered[math.ceil(q*len(ordered))-1],4)
            for key,q in [('p50',.5),('p95',.95),('p99',.99),('max',1.0)]}

def audit(directory):
    summarize = runpy.run_path(str(directory.parents[1]/'berlin-brake-merge/analyze.py'))['summarize']
    trials = summarize(directory)
    active = []
    overruns = []
    for trial in trials:
        with (directory/(trial['tag']+'.csv')).open() as stream:
            rows = list(csv.DictReader(stream))
        # Include the terminal decision and the first rejection, omit latched repeats.
        for row in rows:
            elapsed = float(row['compute_ms'])
            stages = STAGES + (('packing',) if 'packing_ms' in row else ())
            total = sum(float(row[k+'_ms']) for k in stages)
            assert total <= elapsed+.01 and abs(total-elapsed) < 1., (trial['tag'],row['step'],total,elapsed)
            active.append(row)
            if elapsed > 25:
                overruns.append(dict(tag=trial['tag'],step=int(row['step']),ms=elapsed,
                                     progress_m=float(row['progress_m']),reason=row['reason'],
                                     stages={k:[float(row[k+'_ms']),float(row[k+'_cpu_ms'])] for k in stages}))
            if row['mode']=='3':break
    return dict(trials=trials,active_ticks=len(active),overruns=overruns,
                total_ms=quantiles([float(r['compute_ms']) for r in active]),
                stages_ms={s:quantiles([float(r[s+'_ms']) for r in active]) for s in stages})

if __name__ == '__main__':
    directory=Path(sys.argv[1]).resolve()
    print(json.dumps(audit(directory),indent=2))
