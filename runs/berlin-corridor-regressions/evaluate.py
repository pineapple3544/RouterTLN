"""Run frozen Berlin cases using a chosen evaluator. Optional traces for the four regressions."""
import json,sys,subprocess,os
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
BASE=HERE.parent/"berlin-sync-braking/offline-v1"
TARGETS={f"random-{x}" for x in (4135410,6695374,9157059,9285660)}
def run(label,binary,which):
    out=HERE/label;out.mkdir()
    def worker(i):
        spec=json.loads((BASE/f"cases-{i}.json").read_text())
        if which!="all":spec["cases"]=[x for x in spec["cases"] if x["name"] in TARGETS]
        if not spec["cases"]:return
        if which=="trace":
            for x in spec["cases"]:x["checkpoint"]={"ticks":str(out/(x["name"]+".ticks"))}
        cfg=out/f"cases-{i}.json";cfg.write_text(json.dumps(spec))
        with (out/f"worker-{i}.log").open("w") as stream:
            subprocess.run(["taskset","-c",str(2*i),binary,"models/avoid-corridor-v2.pt",str(out/f"results-{i}.jsonl"),str(cfg),"models/race-berlin-wsp.pt"],cwd=ROOT,stdout=stream,stderr=stream,check=True)
    with ThreadPoolExecutor(max_workers=8) as pool:list(pool.map(worker,range(8)))
    (out/"complete").touch()
if __name__=="__main__":run(*sys.argv[1:])
