#!/usr/bin/env python3
"""Summarise a long-run campaign: runs/longrun-<date>/summary.csv and summary.md.

  python3 runs/longrun/summarize.py runs/longrun-20261125

Scoring-side only: uses the physics judge log (ground truth) and the controller CSV. Lap
crossings are counted from the start pose, like scripts/score_drive.py. A run's clean laps are
the laps finished before its first failure (physical contact, stop with speed < 0.05 m/s, latched
emergency, or controller fault). The eval-format numbers additionally only count laps finished
within 180 s of the first control stamp. Nothing in score_drive.py is changed.
"""
import csv
import json
import statistics
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'scripts'))
from score_drive import route_progress  # noqa: E402

EVAL_S = 180.0
DEADLINE_MS = 25.0
BUCKETS = ((1, 5), (6, 10), (11, 20))
FIELDS = ['mode', 'layout', 'obstacles', 'target_laps', 'seed', 'consecutive_clean_laps',
          'laps_within_180s', 'best_lap_s', 'median_lap_s', 'lap1_s', 'stop_reason', 'stop_progress_m',
          'stop_lap', 'collisions', 'deadline_misses', 'max_compute_ms', 'p99_compute_ms',
          'max_along_err_m', 'p99_along_err_m', 'loc_rejections', 'loc_coasts', 'deadline_tolerated', 'budget_brakes',
          'rm_laps', 'rm_best_lap_s', 'rm_hits']


def read(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    return [r for r in rows if None not in r.values()]   # drop a row cut off mid-write


def pct(values, q):
    return float(np.percentile(values, q)) if len(values) else None


def analyse(run_dir, waypoints):
    spec = json.loads((run_dir / 'spec.json').read_text())
    row = dict(mode=spec['mode'], layout=spec['layout'], obstacles=spec['obstacles'],
               target_laps=spec['target_laps'], seed=spec['noise_seed'])
    try:
        controls, judge = read(run_dir / 'run.csv'), read(run_dir / 'run-judge.csv')
    except FileNotFoundError:
        controls, judge = [], []
    if not controls or len(judge) < 2:
        row.update(stop_reason='no_data', consecutive_clean_laps=0, laps_within_180s=0)
        return row, None
    stamps = np.array([float(r['stamp']) for r in controls])
    times = np.array([float(r['stamp']) for r in judge])
    begin = max(0, int(np.searchsorted(times, stamps[0], 'right')) - 1)
    judge, times = judge[begin:], times[begin:]
    progress, length = route_progress(judge, waypoints)
    start = stamps[0]

    crossings = []
    for i in range(len(progress) - 1):
        goal = length * (len(crossings) + 1)
        if progress[i] < goal <= progress[i + 1]:
            crossings.append(times[i] + (times[i + 1] - times[i]) * (goal - progress[i]) / (progress[i + 1] - progress[i]))

    # first failure of each kind
    hit = [t for t, r in zip(times, judge) if r['collision'] != 'none']
    events = sum(1 for a, b in zip(judge, judge[1:]) if a['collision'] == 'none' and b['collision'] != 'none')
    events += judge[0]['collision'] != 'none'
    end_target = crossings[spec['target_laps'] - 1] if len(crossings) >= spec['target_laps'] else times[-1]
    stop = [t for t, p, r in zip(times, progress, judge) if t < end_target and p > .5 and abs(float(r['speed'])) < .05]
    emergency = next(((float(r['stamp']), r['reason']) for r in controls if r['mode'] == '3'), None)
    fault = None
    log = run_dir / 'run-controller.log'
    if log.exists():
        for line in log.read_text(errors='replace').splitlines():
            if 'control_fault=' in line:
                stamp = float(line.split('fault_stamp=')[1].split()[0]) if 'fault_stamp=' in line else start
                fault = (stamp, line.split('control_fault=')[-1].strip())
                break
    candidates = []
    if hit:
        candidates.append((hit[0], 'collision'))
    if emergency:
        candidates.append(emergency)
    if fault:
        candidates.append(fault)
    if stop:
        candidates.append((stop[0], 'stopped'))
    first = min(candidates, default=None)
    fail_t = first[0] if first else float('inf')

    clean = [t for t in crossings[:spec['target_laps']] if t < fail_t]
    laps = np.diff([start, *clean]).tolist()
    within = [t for t in clean if t - start <= EVAL_S]
    eval_laps = laps[:len(within)] if spec['target_laps'] == 10 else laps
    if first:
        reason = first[1]
        stop_p = float(np.interp(fail_t, times, progress))
    elif len(crossings) >= spec['target_laps']:
        reason, stop_p = 'completed', float(progress[-1])
    else:
        reason, stop_p = 'budget', float(progress[-1])

    compute = np.array([float(r['compute_ms']) for r in controls])
    row['budget_brakes'] = sum(r['reason'] == 'budget_brake' for r in controls)
    row.update(consecutive_clean_laps=len(laps), laps_within_180s=len(within),
               best_lap_s=round(min(eval_laps), 3) if eval_laps else None,
               median_lap_s=round(statistics.median(laps[1:]), 3) if len(laps) > 1 else None,
               lap1_s=round(laps[0], 3) if laps else None, stop_reason=reason,
               stop_progress_m=round(stop_p, 1), stop_lap=int(stop_p // length) + 1 if first else None,
               collisions=events, deadline_misses=int((compute > DEADLINE_MS).sum()),
               max_compute_ms=round(float(compute.max()), 2), p99_compute_ms=round(pct(compute, 99), 2))

    rm = sorted(run_dir.glob('race_result_*.json'))
    if rm:
        r = json.loads(rm[-1].read_text())
        row.update(rm_laps=r.get('laps_completed'), rm_best_lap_s=r.get('best_lap'),
                   rm_hits=(r.get('collisions') or {}).get('total'))

    # per-lap series for drift analysis (clean laps only)
    bounds = [start, *clean]
    ctrl_xy = np.array([(float(r['x']), float(r['y'])) for r in controls])
    gt_xy = np.array([(float(r['x']), float(r['y'])) for r in judge])
    gt_at = np.stack([np.interp(stamps, times, gt_xy[:, 0]), np.interp(stamps, times, gt_xy[:, 1])], 1)
    pose_err = np.linalg.norm(ctrl_xy - gt_at, axis=1)
    gt_yaw = np.unwrap([float(r['yaw']) for r in judge])
    yaw_at = np.interp(stamps, times, gt_yaw)
    along = np.abs((ctrl_xy[:, 0] - gt_at[:, 0]) * np.cos(yaw_at) + (ctrl_xy[:, 1] - gt_at[:, 1]) * np.sin(yaw_at))
    before = along[stamps < fail_t] if np.isfinite(fail_t) else along
    if len(before):
        row.update(max_along_err_m=round(float(before.max()), 3), p99_along_err_m=round(pct(before, 99), 3))
    if log.exists():
        text = log.read_text(errors='replace')
        row.update(loc_rejections=text.count('localizer rejected scan'), loc_coasts=text.count('coasting on motion prior'),
                   deadline_tolerated=text.count(' tolerated ('))
    cells = np.array([float(r['remembered_cells']) for r in controls])
    per_lap = []
    for k in range(len(clean)):
        m = (stamps >= bounds[k]) & (stamps < bounds[k + 1])
        per_lap.append(dict(lap=k + 1, time=laps[k], p99=pct(compute[m], 99), cells=float(cells[m].max()) if m.any() else None,
                            pose_err=float(pose_err[m].mean()) if m.any() else None))
    return row, dict(per_lap=per_lap, failure=first and dict(reason=first[1], lap=row['stop_lap'],
                                                              progress_m=row['stop_progress_m']))


def fmt(v):
    return '' if v is None else f'{v:.3f}' if isinstance(v, float) else str(v)


def main():
    root = Path(sys.argv[1])
    waypoints = json.loads((Path(__file__).resolve().parents[2] / 'maps/berlin/global_waypoints.json')
                           .read_text())['global_traj_wpnts_iqp']['wpnts']
    rows, series = [], []
    # only finished runs (run-score.json is written last); skips a run that is still in progress
    for run_dir in sorted(p for p in root.glob('*/*') if (p / 'spec.json').exists() and (p / 'run-score.json').exists()):
        row, extra = analyse(run_dir, waypoints)
        rows.append(row)
        series.append((row, extra))
    rows.sort(key=lambda r: (r['mode'], r['target_laps'], r['layout'], r['obstacles'], r['seed']))
    with open(root / 'summary.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, FIELDS, extrasaction='ignore')
        w.writeheader()
        w.writerows(rows)

    md = [f'# Long-run matrix — {root.name}', '',
          'Generated by `runs/longrun/summarize.py`. Clean laps = laps before the first contact, stop, '
          'emergency or controller fault. `laps_within_180s` and (for 10-lap runs) `best_lap_s` only '
          'count clean laps finished within 180 s of the first control stamp. `median_lap_s` excludes lap 1. '
          '`rm_*` columns are the official race_manager scorer (Mode R only).', '']
    md.append('| ' + ' | '.join(FIELDS) + ' |')
    md.append('|' + '---|' * len(FIELDS))
    for r in rows:
        md.append('| ' + ' | '.join(fmt(r.get(k)) for k in FIELDS) + ' |')

    # aggregates per mode / target / obstacle count
    md += ['', '## Aggregates', '', '| mode | laps | obstacles | runs | full clean | mean clean laps | '
           'mean laps ≤180 s | best lap (min) | median lap (median) |', '|---|---|---|---|---|---|---|---|---|']
    groups = {}
    for r in rows:
        groups.setdefault((r['mode'], r['target_laps'], r['obstacles']), []).append(r)
    for (mode, laps, obs), g in sorted(groups.items()):
        full = sum(r['consecutive_clean_laps'] >= laps for r in g)
        bests = [r['best_lap_s'] for r in g if r.get('best_lap_s')]
        meds = [r['median_lap_s'] for r in g if r.get('median_lap_s')]
        md.append(f'| {mode} | {laps} | {obs} | {len(g)} | {full}/{len(g)} | '
                  f'{statistics.mean(r["consecutive_clean_laps"] for r in g):.1f} | '
                  f'{statistics.mean(r["laps_within_180s"] for r in g):.1f} | '
                  f'{fmt(min(bests) if bests else None)} | {fmt(statistics.median(meds) if meds else None)} |')

    md += ['', '## Failures', '', '| mode | layout | obstacles | laps | seed | reason | lap | progress m |',
           '|---|---|---|---|---|---|---|---|']
    for row, extra in sorted(series, key=lambda x: (x[0]['mode'], x[0]['target_laps'], x[0]['layout'], x[0]['obstacles'])):
        if extra and extra['failure']:
            f = extra['failure']
            md.append(f"| {row['mode']} | {row['layout']} | {row['obstacles']} | {row['target_laps']} | {row['seed']} | "
                      f"{f['reason']} | {f['lap']} | {f['progress_m']} |")
        elif row['stop_reason'] in ('budget', 'no_data'):
            md.append(f"| {row['mode']} | {row['layout']} | {row['obstacles']} | {row['target_laps']} | {row['seed']} | "
                      f"{row['stop_reason']} | | {row.get('stop_progress_m', '')} |")

    md += ['', '## Drift by lap bucket (clean laps, lap 1 excluded from lap time)', '',
           '| mode | bucket | laps | mean lap s | mean compute p99 ms | mean max remembered_cells | '
           'mean pose error m | failures starting in bucket |', '|---|---|---|---|---|---|---|---|']
    for mode in sorted({r['mode'] for r in rows}):
        for lo, hi in BUCKETS:
            pl = [p for row, extra in series if row['mode'] == mode and extra
                  for p in extra['per_lap'] if lo <= p['lap'] <= hi]
            fails = sum(1 for row, extra in series if row['mode'] == mode and extra and extra['failure']
                        and extra['failure']['lap'] and lo <= extra['failure']['lap'] <= hi)
            exposure = sum(1 for row, extra in series if row['mode'] == mode and row['target_laps'] >= lo)
            times = [p['time'] for p in pl if p['lap'] > 1]

            def mean(key):
                v = [p[key] for p in pl if p[key] is not None]
                return f'{statistics.mean(v):.3f}' if v else ''
            md.append(f"| {mode} | {lo}–{hi} | {len(pl)} | {statistics.mean(times):.3f} | {mean('p99')} | "
                      f"{mean('cells')} | {mean('pose_err')} | {fails} of {exposure} runs |" if times else
                      f'| {mode} | {lo}–{hi} | 0 | | | | | {fails} of {exposure} runs |')
    (root / 'summary.md').write_text('\n'.join(md) + '\n')
    print('\n'.join(md))


if __name__ == '__main__':
    main()
