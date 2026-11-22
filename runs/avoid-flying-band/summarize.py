"""Summarize native gym and ROS trials without treating pre-exposure stops as speed failures."""
from pathlib import Path
import csv,json,statistics,collections
p=Path(__file__).resolve().parent
length=json.loads((p.parents[1]/'maps/berlin/global_waypoints.json').read_text())['global_traj_wpnts_iqp']['wpnts'][-1]['s_m']
def csvout(file,rows):
    with file.open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=rows[0]);w.writeheader();w.writerows(rows)
def ros():
    out=[]
    for file in sorted(p.glob('*.csv')):
        if file.stem in ('ros-results','sync-results'):continue
        tag,lo=file.stem.rsplit('-',1); lo=float(lo)
        hi=min(3,lo+.4) if lo<3 else lo+.3
        if lo==2.7:hi=3
        if tag.startswith('wall-ros'):hi=lo+.4
        meta=p/(file.stem+'-settings.json')
        if meta.exists():
            config=json.loads(meta.read_text());assert config['low']==lo
            hi=config['high']
        with file.open() as f:a=list(csv.DictReader(f))
        good=a and float(a[-1]['progress_m'])>=3*length-.01 and a[-1]['mode']=='0' and a[-1]['valid']=='0'
        run=a[:-1] if good else a
        bad=next((x for x in run if x['mode']=='3' or x['valid']!='1' or float(x['target_speed'])<=0 or (float(x['progress_m'])>.5 and float(x['speed'])<.05)),None)
        witness=p/(file.stem+'-contact.log')
        kind=''
        if witness.exists():
            for line in witness.read_text().splitlines():
                if line.strip() in ('data: wall','data: obstacle'):kind=line.split(':')[-1].strip()
        first_hit=next((i for i,x in enumerate(a) if x['collision']!='0'),None)
        hit=bool(kind) or first_hit is not None
        steps=[x for x in run if x['mode']=='1' and x['plan_ready']=='1' and x['valid']=='1']
        assert all(lo-.001<=float(x['target_speed'])<=hi+.001 for x in steps)
        laps=[];t=float(a[0]['stamp'])
        for i in range(1,4):
            row=next((x for x in a if float(x['progress_m'])>=length*i-.01),None)
            if row:t2=float(row['stamp']);laps.append(round(t2-t,3));t=t2
        model='damp' if tag.startswith('wsp') else 'preview' if tag.startswith('preview') else 'high-WSP' if tag.startswith('impact') else 'original'
        out.append(dict(tag=file.stem,model=model,low=lo,high=round(hi,2),exposed=bool(steps),passed=bool(good and not bad and not hit),
                        collision=hit,contact_kind=kind,
                        last_v_before_contact=a[first_hit-1]['speed'] if first_hit is not None and first_hit>0 else '',
                        reason=bad['reason'] if bad else ('finish' if good else 'timeout'),
                        first_reject_m=bad['progress_m'] if bad else '',v_at_reject=bad['speed'] if bad else '',
                        peak_flying_avoid=max((float(x['speed']) for x in steps),default=None),
                        laps=laps,planned_entries=a[-1]['planned_entries']))
    csvout(p/'ros-results.csv',out)
    return out
def sync():
    out=[]
    for file in sorted(p.glob('*.jsonl')):
        xs=list(map(json.loads,file.read_text().splitlines()))
        for x in xs:
            reject=x.get('first_rejection') or {};hit=x.get('contact') or {};tr=x.get('avoid_trace',[])
            steps=[z for z in tr if z['target_v']>0]
            ok=x['reason']=='completed' and len(x['laps'])==3 and not reject and not hit and x['min_speed_after_first_half_m']>=.05
            out.append(dict(batch=file.stem,case=x['case'],seed=x['spec']['seed'],low=x['spec']['sim_avoid_min'],high=x['spec']['sim_avoid_max'],
                    passed=ok,result=x['reason'],completed_laps=len(x['laps']),collision=bool(hit),gym_contact=bool(hit.get('gym_collision')),contact_kind=hit.get('kind',''),pre_contact_v=hit.get('pre_step_v',''),
                    first_rejection=reject.get('reason',''),v_at_reject=reject.get('speed_mps',''),s_at_reject=reject.get('progress_m',''),
                    at_obstacles=x['at_obstacles_mps'],mean_flying_avoid=statistics.mean([z['v'] for z in steps]) if steps else None,
                    times=[round(l['seconds'],3) for l in x['laps']]))
    csvout(p/'sync-results.csv',out)
    return out
if __name__=='__main__':ros();sync()
