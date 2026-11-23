"""Start independent ROS simulator and controller processes for each cold-start trial."""
import json
import os
import runpy
import subprocess
import sys
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor

D=Path(__file__).resolve().parent
TAGS=[f"random-{x}" for x in (1846076,2238863,8181110,9699447,9701112)]

def group(out, index):
    layouts=json.loads((D.parent/"berlin-forward-holdout-100/layouts.json").read_text())
    names=TAGS[index::2]
    sys.argv=[sys.argv[0],str(out),"--spread-cores","--repeats","2"]
    runpy.run_path(str(D.parent/"berlin-brake-merge/ros.py"))["main"](
        {k:layouts[k] for k in names}, domain_base=80+8*index)

if __name__=="__main__":
    out=Path(sys.argv[1]).resolve()
    if len(sys.argv)>2:
        group(out,int(sys.argv[2]))
    else:
        out.mkdir()
        def run(index):
            with (out/f"group{index}.log").open("w") as stream:
                subprocess.run(["taskset","-c",("0,2,4,6","8,10,12,14")[index],
                                sys.executable,__file__,str(out/f"group{index}"),str(index)],
                               stdout=stream,stderr=stream,check=True)
        with ThreadPoolExecutor(max_workers=2) as pool:list(pool.map(run,range(2)))
        (out/"complete").touch()
