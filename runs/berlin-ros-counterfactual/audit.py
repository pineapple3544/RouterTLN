from prepare import *
import hashlib
d=D
def lines(p):return [json.loads(l) for l in p.read_text().splitlines()]
out={}
for k in ['failed','passed']:
    hist=json.loads((d/(k+'.json')).read_text())['controls']
    shadow=lines(d/(k+'-shadow-0.jsonl'))
    assert len(hist)==len(shadow)
    v=dict(ticks=len(hist),mode_matches=sum(u['mode']==r['mode'] for r,u in zip(hist,shadow)),
           side_matches=sum(u['side']==r['side'] for r,u in zip(hist,shadow)),
           count_matches=sum(u['stored']==r['stored'] for r,u in zip(hist,shadow)),
           max_steer_error=max(abs(u['command'][0]-r['steer']) for r,u in zip(hist,shadow)),
           max_speed_command_error=max(abs(u['command'][1]-r['target']) for r,u in zip(hist,shadow)))
    assert v['mode_matches']==v['side_matches']==v['count_matches']==len(hist)
    assert v['max_steer_error']<1e-6 and v['max_speed_command_error']<1e-5
    out[k]=v
out['forks']=[]
for i in [898,908,915]:
    digest=hashlib.sha256((d/f'failed-{i}.cbor').read_bytes()).hexdigest()
    for arm in ['tln','teacher','one-sync','one-raw']:
        for suffix in ['', '-alt']:
            r=lines(d/f'failed-{i}-{arm}{suffix}.jsonl')
            v=r[0];last=r[-1]
            assert last['reason'] in ('cleared','teacher_no_plan','braking_footprint','no_corridor','collision')
            out['forks'].append(dict(step=i,sha256=digest,arm=arm,alternate=bool(suffix),s=v['s'],
                                   initial_state=v['state'],reason=last['reason'],ticks=len(r),
                                   decision=last['i'],first_tln_rad=v['policy'],first_source_rad=v['source_steer'],
                                   first_applied_rad=v['steer'],first_target=v['target_before_guard'],
                                   gap_m=min((z['min_box_gap'] for z in r if 'min_box_gap' in z),default=None)))
    group=[v for v in out['forks'] if v['step']==i]
    assert all(v['initial_state']==group[0]['initial_state'] for v in group if not (i==908 and v['arm']=='one-sync'))
    assert len({v['sha256'] for v in group})==1
p=ROOT/'runs/berlin-forward-holdout-100/hashes.json'
for path,want in json.loads(p.read_text()).items():
    assert hashlib.sha256((ROOT/path).read_bytes()).hexdigest()==want,path
out['brake_variants']=[{k:v for k,v in row.items() if k!='trace'} for row in json.loads((d/'brake-variants.json').read_text())]
(d/'summary.json').write_text(json.dumps(out,indent=2)+'\n')
with (d/'forks.csv').open('w',newline='') as h:
    w=csv.DictWriter(h,out['forks'][0].keys());w.writeheader();w.writerows(out['forks'])
print(out['failed'],out['passed'])
