from pathlib import Path
import bisect,csv,collections,json,math,statistics as st,sys
root=Path(sys.argv[1]).resolve()
arms=sys.argv[2:] or ['local','half','full','both','continuity','rescue','coupled','graduated']
wp=json.loads((Path('/workspace/maps/berlin/global_waypoints.json')).read_text())['global_traj_wpnts_iqp']['wpnts']
length=wp[-1]['s_m']+math.hypot(wp[-1]['x_m']-wp[0]['x_m'],wp[-1]['y_m']-wp[0]['y_m'])
def clean(r):
    return r['reason']=='time_limit' and abs(r['time_s']-180)<1e-6 and r['contact'] is None and r['first_rejection'] is None and r['min_speed_after_first_half_m']>=.05 and bool(r['laps'])
def read_arm(arm):
    raw={r['case']:r for p in root.glob(arm+'-?.jsonl') for r in map(json.loads,p.read_text().splitlines())}
    assert len(raw)==54,(arm,len(raw))
    return raw
def inspect(r):
    path=root/Path(r['spec']['phase_trace']).name
    with path.open() as stream:
        data=[(float(x['s']),float(x['t']),int(x['lap']),int(x['mode']),int(x['adopted']),float(x['prev_interval_gap_m']),float(x['entry_blend']),int(x['entry_checked'])) for x in csv.DictReader(stream)]
    ss=list(x[0] for x in data)
    # Monotone station for locating first forward crossings; stalled failure tails are discarded.
    for i in range(1,len(ss)):ss[i]=max(ss[i],ss[i-1])
    def time_at(s):
        i=bisect.bisect_right(ss,s)-1
        return data[i][1]+(s-ss[i])/(ss[i+1]-ss[i])*(data[i+1][1]-data[i][1])
    n=len(r['laps'])
    flying=[x for x in data if 2<=x[2]<=n and x[3]==1]
    counts=collections.Counter(x[6] for x in flying if x[7]);passed={}
    stations=sorted(r['stations_m'])
    for j,first in enumerate(stations):
        if len(stations)==1:continue
        gap=(stations[(j+1)%len(stations)]-first)%length
        if not .5<gap<=5:continue
        for lap in range(2,n+1):
            start=(lap-1)*length+first-1;end=start+1+gap+.5
            if start<length or end>n*length or end>=ss[-1]:continue
            i=bisect.bisect_right(ss,start)-1;jj=bisect.bisect_left(ss,end)
            A=D=0.;gapmin=float('inf')
            for k in range(i,jj):
                a,b=data[k],data[k+1]
                dt=max(0,min(b[1],time_at(end))-max(a[1],time_at(start)))
                if dt<=0:continue
                # Includes both partly overlapping boundary intervals conservatively.
                gapmin=min(gapmin,b[5])
                if a[3]==1:A+=dt;D+=dt*a[4]
            passed[(j,lap)]={'sec':time_at(end)-time_at(start),'gap':gapmin,'A':A,'D':D}
    return dict(ok=clean(r),laps=n,best=min((l['seconds'] for l in r['laps']),default=None),gap=r['min_sat_gap_m'],
                flysec=st.mean(v['seconds'] for v in r['laps'][1:]) if n>1 else None,A=len(flying),D=sum(x[4] for x in flying),
                counts=counts,passed=passed,over=r['control_wall']['over_25ms'],contact=r['contact'] is not None,
                rejection=(r['first_rejection'] or {}).get('reason',r['reason']),obs=r['spec']['count'])
def aggregate(d,keys):
    ss=[d[k] for k in keys]
    return sum(v['D'] for v in ss)*100/sum(v['A'] for v in ss)
raw={a:read_arm(a) for a in arms}
ref=raw['both'] if 'both' in arms else next(iter(raw.values()))
data={}; passes=[]
for a,rows in raw.items():
    assert set(rows)==set(ref)
    data[a]={}
    for k,r in rows.items():
        other=ref[k];assert r['laps'][:1]==other['laps'][:1] and r['seed']==other['seed'] and r['boxes']==other['boxes'] and r['pose_source']=='lidar_static_map'
        spec=lambda row:{key:value for key,value in row['spec'].items() if key not in ('cluster_entry','entry_candidates','phase_trace')}
        assert spec(r)==spec(other)
        v=inspect(r);data[a][k]=v
        for (j,lap),p in v['passed'].items():passes.append(dict(arm=a,case=k,obstacle_index=j,lap=lap,**p))
common=sorted(k for k in ref if all(d[k]['ok'] for d in data.values()))
common_pass={k:set.intersection(*(set(d[k]['passed']) for d in data.values())) for k in common}
def ratios(d,pairs):
    each=[];merged=[]
    for k,keys in pairs.items():
        if not keys:continue
        pp=[d[k]['passed'][key] for key in keys]; each.append(st.mean(p['sec'] for p in pp));merged+=pp
    return dict(cases=len(each),passes=len(merged),sec=st.mean(each),gap=min(p['gap'] for p in merged),adopt=100*sum(p['D'] for p in merged)/sum(p['A'] for p in merged))
suites={k:set(v) for k,v in json.loads(Path('/opt/berlin-cases/suites.json').read_text()).items()}
assert len(suites['verified'])==24 and len(suites['holdout'])==30 and suites['verified'].isdisjoint(suites['holdout'])
summary={}
for a,d in data.items():
    refd=data.get('both',d);paired=[k for k,v in d.items() if v['ok'] and refd[k]['ok']]
    pairs={k:set(d[k]['passed']) & set(refd[k]['passed']) for k in paired}
    basepairs=ratios(refd,pairs);vs=ratios(d,pairs)
    allgap=min(v['gap'] for v in d.values())
    summary[a]=dict(clean=sum(v['ok'] for v in d.values()),contact=sum(v['contact'] for v in d.values()),
      fails={k:v['rejection'] for k,v in d.items() if not v['ok']},common=len(common),best=st.mean(d[k]['best'] for k in common),
      flying=st.mean(d[k]['flysec'] for k in common),gap=allgap,adoption=aggregate(d,list(d)),adoption_common=aggregate(d,common),
      suites={label:{'clean':sum(d[k]['ok'] for k in keys),'adopt':aggregate(d,keys),'best_common':st.mean(d[k]['best'] for k in keys if k in common)} for label,keys in suites.items()},
      cases_under_85=sum(100*v['D']<85*v['A'] for v in d.values() if v['A']),over=sum(v['over'] for v in d.values()),blend=dict(sum((v['counts'] for v in d.values()),collections.Counter())),
      cluster=ratios(d,common_pass),paired_clusters=vs,paired_delta_sec=vs['sec']-basepairs['sec'],
      paired_gap_losses_1cm=sum(d[k]['gap']+.01<refd[k]['gap'] for k in paired),
      target023={k:v for k,v in d['pace-3-023'].items() if k not in ('counts','passed')})
with (root/'case-results.csv').open('w',newline='') as f:
    writer=csv.writer(f);writer.writerow(['arm','case','obstacles','complete_180s','laps','best_s','flying_lap_s','min_sat_gap_m','flying_tln_pct','control_over_25ms','end_reason'])
    for arm,rows in data.items():
        for name,v in sorted(rows.items()):
            writer.writerow([arm,name,v['obs'],v['ok'],v['laps'],v['best'],v['flysec'],v['gap'],100*v['D']/v['A'] if v['A'] else None,v['over'],v['rejection']])
with (root/'paired-pass.csv').open('w',newline='') as f:
    w=csv.DictWriter(f,fieldnames=list(passes[0]));w.writeheader();w.writerows(passes)
(root/'entry-results.json').write_text(json.dumps(summary,indent=2))
print(json.dumps(summary,indent=1))
