from pathlib import Path
import csv,subprocess,sys
p=Path('runs/avoid-flying-band')
base='models/avoid-corridor-v2.pt'
wsp=str(p/'high-preview-1.0.pt')
def run(tag,model,lo,hi):
    subprocess.run([sys.executable,str(p/'ros.py'),model,tag,f'{lo},{hi}'],check=True)
    with (p/(tag+f'-{lo:.1f}.csv')).open() as f:rows=list(csv.DictReader(f))
    return any(x['plan_ready']=='1' and x['mode']=='1' for x in rows),any(x['collision']!='0' for x in rows)
for lo in (6.,7.,8.,9.,9.6):
    hi=round(lo+.4,1)
    exposed,hit=run('ceiling-ros',base,lo,hi)
    if not exposed and not hit:exposed,hit=run('ceiling-ros-retry',base,lo,hi)
    if hit and exposed:
        run('ceiling-confirm',base,lo,hi)
        run('impact-ros-confirm',wsp,lo,hi)
        break
