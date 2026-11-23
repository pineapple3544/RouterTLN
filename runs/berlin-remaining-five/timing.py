"""Summarize successful control cycles and the pre-optimization deadline event."""
from pathlib import Path
import csv,json
import numpy as np
D=Path(__file__).resolve().parent
report={}
for f in ['horizon-ros','final-ros']:
    all_rows=[];events=[];section=[]
    for p in (D/f).glob('group*/*.csv'):
        with p.open() as h: rows=list(csv.DictReader(h))
        if not rows:continue
        all_rows.extend(r for r in rows if r['valid']=='1')
        events.extend(dict(trial=p.stem,progress_m=r['progress_m'],compute_ms=r['compute_ms'],corridor_ms=r['corridor_ms'],corridor_cpu_ms=r['corridor_cpu_ms']) for r in rows if r['reason']=='deadline' and float(r['compute_ms'])>25)
        if p.stem.startswith('random-8181110-'):
            section.extend(r for r in rows if 88.8<=float(r['progress_m'])<=89.6 and (r['valid']=='1' or float(r['compute_ms'])>25))
    def stats(rows):
        return {key:dict(zip(['p50','p95','p99','max'],np.percentile([float(x[key]) for x in rows],[50,95,99,100]).round(3))) for key in ('compute_ms','corridor_ms','corridor_cpu_ms')}
    report[f]=dict(valid_cycles=len(all_rows),all_valid=stats(all_rows),sector_8181110=stats(section),events=events)
(D/'timing.json').write_text(json.dumps(report,indent=2)+'\n')
