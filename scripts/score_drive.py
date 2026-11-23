"""Score a finished local simulation from its independent physics log."""
import bisect
import csv
import json
import sys
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree


def read_csv(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def route_progress(samples, route):
    """Project each physics sample onto the nearest adjacent racing-line segment."""
    points = np.array([(p['x_m'], p['y_m']) for p in route])
    end = np.roll(points, -1, axis=0)
    edges = end - points
    lengths = np.linalg.norm(edges, axis=1)
    keep = lengths > 1e-6
    points, edges, lengths = points[keep], edges[keep], lengths[keep]
    starts = np.r_[0., np.cumsum(lengths)[:-1]]
    length = lengths.sum()
    xy = np.array([(float(row['x']), float(row['y'])) for row in samples])
    nearest = cKDTree(points).query(xy)[1]
    ids = (nearest[:, None] + np.array([-2, -1, 0, 1])) % len(edges)
    offsets = xy[:, None, :] - points[ids]
    t = np.clip((offsets * edges[ids]).sum(axis=2) / lengths[ids]**2, 0, 1)
    error = offsets - t[:, :, None] * edges[ids]
    best = (error**2).sum(axis=2).argmin(axis=1)
    station = (starts[ids] + t * lengths[ids])[np.arange(len(xy)), best]
    delta = (np.diff(station) + length/2) % length - length/2
    return np.r_[0., np.cumsum(delta)], length


def score(metrics, physics, waypoints, target, duration=0):
    controls, samples = read_csv(metrics), read_csv(physics)
    if not controls or len(samples) < 2:
        return {'accepted': False, 'failure': 'missing_control_or_judge_samples'}
    times = [float(r['stamp']) for r in samples]
    stamps = [float(r['stamp']) for r in controls]
    if any(b <= a for a, b in zip(times, times[1:])) or times[0] > stamps[0] or times[-1] < stamps[-1]:
        return {'accepted': False, 'failure': 'incomplete_judge_coverage'}
    begin = max(0, bisect.bisect_right(times, stamps[0])-1)
    samples, times = samples[begin:], times[begin:]
    # A frame counter is a count of control intervals containing a physical hit.
    hits = [t for t, r in zip(times, samples) if r['collision'] != 'none']
    prev = float('-inf')
    for i, (row, stamp) in enumerate(zip(controls, stamps)):
        boundary = times[-1] if i+1 == len(controls) else stamp
        row['collision'] = int(bisect.bisect_right(hits, boundary) > bisect.bisect_right(hits, prev))
        prev = boundary
    with metrics.with_suffix('.scored.tmp').open('w') as out:
        writer = csv.DictWriter(out, controls[0].keys())
        writer.writeheader()
        writer.writerows(controls)
    metrics.with_suffix('.scored.tmp').replace(metrics)

    route = json.loads(waypoints.read_text())['global_traj_wpnts_iqp']['wpnts']
    progress, length = route_progress(samples, route)
    start = stamps[0]
    deadline = start + duration if duration else float('inf')
    if duration:
        if target or not np.isfinite(duration) or duration <= 0:
            raise ValueError("timed scoring requires a positive duration and no lap cap")
        progress = progress - np.interp(start, times, progress)
    crossings = []
    for i, (p0, p1) in enumerate(zip(progress, progress[1:])):
        goal = length * (len(crossings)+1)
        if p0 < goal <= p1:
            crossing = times[i] + (times[i+1]-times[i])*(goal-p0)/(p1-p0)
            if crossing > deadline:
                break
            crossings.append(crossing)
    if duration:
        end = deadline
        hits = [t for t in hits if start <= t <= end]
        stops = [t for t, p, sample in zip(times, progress, samples)
                 if start <= t <= end and p > .5 and abs(float(sample['speed'])) < .05]
        emergency = next(((t, row['reason']) for t, row in zip(stamps, controls)
                          if t <= end and row['mode'] == '3'), None)
        fault = None
        log = metrics.with_name(metrics.stem + '-controller.log')
        if log.exists():
            for line in log.read_text().splitlines():
                if 'control_fault=' not in line:
                    continue
                stamp = (float(line.split('fault_stamp=')[1].split()[0])
                         if 'fault_stamp=' in line else start)
                if stamp <= end:
                    fault = (stamp, line.split('control_fault=')[-1].strip())
                    break
        first_failure = min([*hits, *stops, *([emergency[0]] if emergency else []),
                             *([fault[0]] if fault else [])], default=float('inf'))
        coverage = min(times[-1], stamps[-1]) >= end
        clean_times = [t for t in crossings if t < first_failure]
        laps = np.diff([start, *clean_times]).tolist()
        return {
            'accepted': bool(coverage and laps and first_failure == float('inf')),
            'duration_s': duration, 'covered_s': min(duration, min(times[-1], stamps[-1])-start),
            'physical_laps': len(crossings), 'consecutive_laps': len(laps), 'lap_s': laps,
            'best_lap_s': min(laps, default=None), 'collision_samples': len(hits),
            'stopped_samples_before_finish': len(stops), 'emergency': emergency and emergency[1],
            'controller_fault': fault and fault[1], 'failure': None if coverage else 'short_timed_run',
        }
    end = crossings[target-1] if target and len(crossings) >= target else times[-1]
    stops = sum(1 for t, p, row in zip(times, progress, samples)
                if t < end and p > .5 and abs(float(row['speed'])) < .05)
    emergency = next((r['reason'] for r in controls if r['mode']=='3'), None)
    log = metrics.with_name(metrics.stem+'-controller.log')
    fault = next((line.split('control_fault=')[-1].strip() for line in log.read_text().splitlines()
                  if 'control_fault=' in line), None) if log.exists() else None
    return {'accepted': bool(target and len(crossings) >= target and not (hits or stops or emergency or fault)),
            'target_laps': target, 'physical_laps': len(crossings), 'lap_s': np.diff([stamps[0], *crossings]).tolist(),
            'max_progress_m': float(max(progress)), 'lap_length_m': length, 'collision_samples': len(hits),
            'stopped_samples_before_finish': stops, 'controller_fault': fault, 'emergency': emergency}


if __name__ == '__main__':
    metrics, physics, waypoints = map(Path, sys.argv[1:4])
    result = score(metrics, physics, waypoints, int(sys.argv[4]), float(sys.argv[5]) if len(sys.argv)>5 else 0)
    metrics.with_name(metrics.stem+'-score.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result))
    if result.get("failure"):
        sys.exit(2)
