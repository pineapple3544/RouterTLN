"""Export one row per original seed or ROS attempt. Require the complete manifest."""
import csv
import hashlib
import json
import statistics
from collections import Counter
from pathlib import Path

HERE = Path(__file__).resolve().parent
BLIND = HERE.parent/'berlin-blind-300'


def rows(directory):
    return [json.loads(line) for file in directory.glob('results-*.jsonl') for line in file.open()]


def passed(z):
    return (z['reason']=='completed' and len(z['laps'])==3 and z['contact'] is None
            and not z['first_rejection'] and z['min_speed_after_first_half_m']>=.05)


def reason(z):
    return (z['first_rejection'] or {}).get('reason') or z['reason']


def save(name, records):
    with (HERE/name).open('w') as stream:
        writer = csv.DictWriter(stream, records[0].keys())
        writer.writeheader()
        writer.writerows(records)


def by_seed(records):
    result = {z['spec']['name']:z for z in records}
    assert len(result)==len(records)
    return result


def offline():
    records = by_seed(rows(HERE/'verified-300'))
    initial = by_seed(rows(HERE/'frozen-300')) if (HERE/'frozen-300/complete').exists() else records
    old = by_seed(rows(BLIND/'offline'))
    assert set(records)==set(old)==set(initial)==set(json.loads((BLIND/'layouts.json').read_text()))
    table=[]
    for seed,z in sorted(records.items()):
        assert z['pose_source']=='lidar_static_map' and reason(z)==reason(initial[seed]) and passed(z)==passed(initial[seed])
        lap=[r['seconds'] for r in z['laps']]
        table.append(dict(seed=seed,accepted=passed(z),legacy_gt_accepted=passed(old[seed]),reason=reason(z),
                          collision=z['contact'] is not None,progress_m=z['progress_m'],physical_laps=len(lap),
                          min_speed_mps=z['min_speed_after_first_half_m'],
                          **{f'lap{i+1}_s': lap[i] if i<len(lap) else '' for i in range(3)}))
    save('offline.csv',table)
    return dict(trials=len(table), accepted=sum(t['accepted'] for t in table),
                failure_reasons=dict(Counter(t['reason'] for t in table if not t['accepted'])),
                physical_collisions=sum(t['collision'] for t in table),legacy_accepted=sum(t['legacy_gt_accepted'] for t in table),
                newly_failed=sum(not t['accepted'] and t['legacy_gt_accepted'] for t in table),
                newly_passed=sum(t['accepted'] and not t['legacy_gt_accepted'] for t in table),
                accepted_median_lap_s=[statistics.median(t[f'lap{i}_s'] for t in table if t['accepted']) for i in (1,2,3)])


def ros():
    selected=json.loads((BLIND/'preregister.json').read_text())['ros_selected']
    expected={(name,rep) for name in selected for rep in (1,2)}
    table=[]
    for f in sorted((HERE/'ros').rglob('*-score.json')):
        z=json.loads(f.read_text())
        first=f.with_name(f.name.replace('-score.json','.csv'))
        with first.open() as stream:
            ticks=list(csv.DictReader(stream))
        assert not ticks or ticks[0]['plan_ready']=='0'
        ready=next((float(r['progress_m']) for r in ticks if r['plan_ready']=='1'),None)
        lap=z.get('lap_s',[])
        plan_ok=ready is not None and ready>=z['lap_length_m']-.15 and int(ticks[-1]['planned_entries'])>0
        # Verify stopped speed even in the first half metre once physical motion began.
        motion_started = False
        stopped_after_launch = False
        if z.get('accepted'):
            start = float(ticks[0]['stamp'])
            end = start + sum(lap[:3])
            physics = f.with_name(f.name.replace('-score.json','-judge.csv'))
            with physics.open() as stream:
                for frame in csv.DictReader(stream):
                    t = float(frame['stamp'])
                    if t < start:
                        continue
                    if t >= end:
                        break
                    v = abs(float(frame['speed']))
                    motion_started |= v > .1
                    if motion_started and v < .05:
                        stopped_after_launch = True
                        break
        acceptance=(z['accepted'] and plan_ok and not stopped_after_launch and
                    (z['runner_exit']==0 or z.get('post_finish_rescored',False)))
        failure=(z.get('failure') or z.get('controller_fault') or z.get('emergency')
                 or ('collision' if z.get('collision_samples') else 'unverified_finish_or_plan')) if not acceptance else ''
        table.append(dict(seed=z['seed'],repeat=z['repetition'],accepted=acceptance,
                          physical_laps=z.get('physical_laps',''),failure=failure,wrapper_exit=z['runner_exit'],
                          post_finish_rescored=z.get('post_finish_rescored',False),
                          stop_after_launch_on_provisional_pass=stopped_after_launch,
                          stopped_samples=z.get('stopped_samples_before_finish',''),collision_samples=z.get('collision_samples',''),
                          plan_ready_m=ready,planned_entries=ticks[-1]['planned_entries'] if ticks else '',
                          max_compute_ms=max((float(t['compute_ms']) for t in ticks),default=None),
                          **{f'lap{i+1}_s':lap[i] if i<len(lap) else '' for i in range(3)}))
    assert len(table)==len(expected) and {(t['seed'],t['repeat']) for t in table}==expected
    save('ros.csv',table)
    return dict(trials=len(table),accepted=sum(t['accepted'] for t in table),
                both_repeats_passed=sum(all(t['accepted'] for t in table if t['seed']==seed) for seed in selected),
                collision_trials=sum(bool(t['collision_samples']) for t in table),
                failure_reasons=dict(Counter(t['failure'] for t in table if not t['accepted'])),
                accepted_median_lap_s=[statistics.median(t[f'lap{i}_s'] for t in table if t['accepted']) for i in (1,2,3)])


def comparison():
    multi=by_seed(rows(HERE/'pilot'))
    single=by_seed([json.loads(line) for line in (HERE/'single-network.jsonl').open()])
    assert set(multi)==set(single) and len(multi)==5
    save('policy-comparison.csv',[dict(seed=k,two_networks=reason(multi[k]),single_network=reason(single[k]),
                                        two_laps=len(multi[k]['laps']),single_laps=len(single[k]['laps']))
                                  for k in sorted(multi)])


if __name__=='__main__':
    comparison()
    result=dict(offline=offline(),ros=ros(),manifest_sha256=hashlib.sha256((BLIND/'preregister.json').read_bytes()).hexdigest())
    (HERE/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
