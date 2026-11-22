from pathlib import Path
import json
p=Path(__file__).resolve().parent
levels=[(0,0),(1.2,1.6),(1.5,1.9),(1.8,2.2),(2.1,2.5),(2.4,2.8),(2.7,3.0)]
def cases(name,levels,seeds,layout='original'):
    w=json.loads(Path('maps/berlin/global_waypoints.json').read_text())['global_traj_wpnts_iqp']['wpnts']
    first,stride,offsets={'original':(94,30,[.083,-.083,.083]),'mirror':(94,30,[-.083,.083,-.083]),'tight':(124,18,[.12,-.12,.12]),'same-side':(304,32,[.12]*3)}[layout]
    rows=[dict(name=f'{layout}-{seed}-{lo}',map='berlin',layout=0,start=37,count=3,seed=seed,
               stations=[w[first+i*stride]['s_m']-w[37]['s_m'] for i in range(3)],
               shapes=[[.3,.3,0]]*3,offsets=offsets,full_lap=True,laps=3,lap_planning=True,
               speed_cap=max(3,hi),sim_avoid_min=lo,sim_avoid_max=hi,trace_avoid=True)
          for lo,hi in levels for seed in seeds]
    (p/(name+'.json')).write_text(json.dumps(dict(teacher=False,cases=rows),indent=2))
if __name__=='__main__':cases('sweep',levels,[2028,7719])
