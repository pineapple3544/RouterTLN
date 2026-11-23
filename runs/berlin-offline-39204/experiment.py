"""Capture the Berlin 39204 third-lap failure and branch from its saved prefix.
Run inside el-native-dev from /workspace. The output directory must be new.
"""
import argparse
import collections
import csv
import json
import math
import os
from pathlib import Path
import subprocess
from concurrent.futures import ThreadPoolExecutor

import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
LATERAL = [-.06, -.03, 0, .03, .06]
HEADING = [-6, -3, 0, 3, 6]
SPEED = [0, -.15, .15]


def records(path):
    with path.open() as stream:
        return [json.loads(line) for line in stream]


def evaluate(folder, label, cases):
    config = folder/(label+".json")
    result = folder/(label+".jsonl")
    config.write_text(json.dumps(dict(device="cpu", teacher=False, cases=cases), indent=2)+"\n")
    with (folder/(label+".log")).open("w") as log:
        subprocess.run(["bash", "scripts/evaluate_avoid.sh", "models/avoid-corridor-v2.pt",
                        str(result), str(config), "models/race-berlin-wsp.pt"],
                       cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, check=True)
    return records(result)


def main(folder):
    folder.mkdir(parents=True)
    base = json.loads((HERE/"layout.json").read_text())
    snap = str(folder/"approach.cbor")
    case = dict(base, checkpoint=dict(save=snap, from_m=111, lookback_s=.65,
                                      ticks=str(folder/"original-ticks.jsonl")))
    original, = evaluate(folder, "capture", [case])
    rejection = original["first_rejection"]
    assert original["snapshot_written"] and rejection["reason"] == "no_corridor"
    assert len(original["laps"])==2
    actual = records(folder/"original-ticks.jsonl")
    failed = next(t for t in actual if t["reason"]=="no_corridor")
    assert all(2.7-1e-5<=t["command"][1]<=3.+1e-5 for t in actual if t["mode"]==1 and t["ready"])
    assert [round((b["t"]-a["t"])*1000) for a,b in zip(actual[:10],actual[1:11])]==[20,30]*5
    prior = ROOT/"runs/berlin-seed-repair/retest-new"
    ros_poses = next(yaml.safe_load_all((prior/"random-39204-2-poses.yaml").read_text()))["poses"]
    for b, p in zip(original["boxes"],ros_poses,strict=True):
        assert math.hypot(b[0]-p["position"]["x"],b[1]-p["position"]["y"])<1e-5
        angle = 2*math.atan2(p["orientation"]["z"],p["orientation"]["w"])
        assert abs(math.remainder(b[2]-angle,math.tau))<1e-5
        assert max(abs(b[3]-.3),abs(b[4]-.3))<1e-6
    with (prior/"random-39204-2.csv").open() as f:
        ros = next(r for r in csv.DictReader(f) if r["reason"]=="no_corridor")

    def sweep(dv):
        cases=[]
        for d in LATERAL:
            for h in HEADING:
                params=dict(lateral_m=d, yaw_rad=math.radians(h), speed_mps=dv)
                cp=dict(restore=snap, perturb=params)
                if dv==0 and h==0 and d==0:
                    cp["ticks"]=str(folder/"unchanged-ticks.jsonl")
                cases.append(dict(base,name=f"lat{d:+.2f}_yaw{h:+}_v{dv:+.2f}",checkpoint=cp))
        return evaluate(folder, "sweep"+str(SPEED.index(dv)), cases)

    with ThreadPoolExecutor(max_workers=3) as pool:
        branches=[r for batch in pool.map(sweep,SPEED) for r in batch]
    assert len(branches)==75
    restored=records(folder/"unchanged-ticks.jsonl")
    assert restored and restored==[r for r in actual if r["step"]>=restored[0]["step"]]
    control=next(b for b in branches if not any(b["spec"]["checkpoint"]["perturb"].values()))
    for k in ("reason","contact","first_rejection","events","laps","progress_m","time_s","min_sat_gap_m"):
        assert original[k]==control[k], k
    with (folder/"conditions.csv").open("w",newline="") as stream:
        writer=csv.writer(stream)
        writer.writerow(["lateral_delta_m","heading_delta_deg","speed_delta_mps","outcome","stop_progress_m","laps"])
        counts=collections.Counter()
        for b in branches:
            q=b["spec"]["checkpoint"]["perturb"]
            why=b["first_rejection"] or {}
            outcome=why.get("reason",b["reason"])
            counts[outcome]+=1
            writer.writerow([q["lateral_m"],round(math.degrees(q["yaw_rad"])),q["speed_mps"],outcome,why.get("progress_m"),len(b["laps"])])
    report=dict(ros_stop_m=float(ros["progress_m"]), offline_stop_m=rejection["progress_m"],
                ros_xy=[float(ros["x"]),float(ros["y"])], offline_xy=failed["car"][:2],
                position_difference_m=math.dist(failed["car"][:2],[float(ros["x"]),float(ros["y"])]),
                empty_stage=failed["first_empty"],entry=control["branch_entry"],failed_step=failed["step"],
                seconds_before_failure=failed["t"]-control["branch_entry"]["stamp"],matched_ticks=len(restored),
                physics_period_ms=10,control_period_ms=original["control_ms"],boxes=original["boxes"],outcomes=dict(counts))
    (folder/"summary.json").write_text(json.dumps(report,indent=2)+"\n")
    print(json.dumps(report))

if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output",type=Path)
    main(parser.parse_args().output.resolve())
