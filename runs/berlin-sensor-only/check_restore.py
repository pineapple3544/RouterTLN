"""Check a restored approach against uninterrupted control ticks on the same build."""
import json, subprocess, tempfile
from pathlib import Path
D=Path(__file__).resolve().parent
with tempfile.TemporaryDirectory() as tmp:
    folder=Path(tmp)
    for f in (D.parent/'berlin-remaining-five/final-100').glob('cases-*.json'):
        settings=json.loads(f.read_text())
        case=next((s for s in settings['cases'] if s['name']=='random-1846076'),None)
        if case:break
    settings['cases']=[case]
    common=folder/'state.cbor'
    out=[]
    for tag,options in [('full',dict(save=str(common),at_m=20.4)),('restore',dict(restore=str(common)))]:
        ticks=folder/(tag+'.jsonl')
        case['checkpoint']=dict(options,ticks=str(ticks))
        cfg=folder/(tag+'.json');cfg.write_text(json.dumps(settings))
        rows=folder/(tag+'-results.jsonl')
        subprocess.run(['/build/el_eval_avoid','models/avoid-corridor-v2.pt',str(rows),str(cfg),'models/race-berlin-wsp.pt'],cwd=D.parents[1],stdout=subprocess.DEVNULL,check=True)
        result=json.loads(rows.read_text())
        assert result['first_rejection'] is None and result['contact'] is None and len(result['laps'])==3
        out.append({r['step']:r for line in ticks.open() if (r:=json.loads(line))})
    assert out[1] and all(out[0][k]==v for k,v in out[1].items())
    print('restored ticks identical', len(out[1]), 'first',min(out[1]),'last',max(out[1]))
