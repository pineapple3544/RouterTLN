"""Run frozen Berlin cases using a chosen evaluator. Optional traces for the five failures."""
import json,sys,subprocess,os
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
BASE=HERE/"frozen"
TARGETS={f"random-{x}" for x in (1846076,2238863,8181110,9699447,9701112)}
def run(label,binary,which):
    out=HERE/label;out.mkdir()
    cores=[int(core) for core in os.environ.get("EVAL_CORES","0,2,4,6,8,10,12,14").split(",")]
    def worker(i):
        spec=json.loads((BASE/f"cases-{i}.json").read_text())
        if which!="all":spec["cases"]=[x for x in spec["cases"] if x["name"] in TARGETS]
        if not spec["cases"]:return
        if which=="trace":
            for x in spec["cases"]:x["checkpoint"]={"ticks":str(out/(x["name"]+".ticks"))}
        cfg=out/f"cases-{i}.json";cfg.write_text(json.dumps(spec))
        with (out/f"worker-{i}.log").open("w") as stream:
            subprocess.run(["taskset","-c",str(cores[i%len(cores)]),binary,"models/avoid-corridor-v2.pt",str(out/f"results-{i}.jsonl"),str(cfg),"models/race-berlin-wsp.pt"],cwd=ROOT,stdout=stream,stderr=stream,check=True)
    with ThreadPoolExecutor(max_workers=len(cores)) as pool:list(pool.map(worker,range(8)))
    (out/"complete").touch()
if __name__=="__main__":run(*sys.argv[1:])
