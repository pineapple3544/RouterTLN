"""Audit closed-loop Berlin results from ROS ground truth, controller and metrics."""
import csv
import hashlib
import json
import runpy
from pathlib import Path
from statistics import mean

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
summarize = runpy.run_path(str(HERE.parent / "berlin-brake-merge/analyze.py"))["summarize"]


def evaluate(groups):
    cases = {}
    for group in groups:
        folder = HERE/group
        cfg = json.loads((folder/"layouts.json").read_text())
        trials = summarize(folder)
        assert len(trials)==2*len(cfg)
        for name, placement in cfg.items():
            subset = [t for t in trials if t["tag"].rsplit("-",1)[0]==name]
            assert len(subset)==2
            for t in subset:
                with (folder/(t["tag"]+".csv")).open() as stream:
                    rows=list(csv.DictReader(stream))
                log=(folder/(t["tag"]+"-controller.log")).read_text()
                assert "TLN inference device: cpu" in log
                t["timeout"] = any("timeout" in row["reason"] for row in rows) or "timeout:" in log
                t["initial_plan"] = [int(rows[0][k]) for k in ("plan_ready","planned_entries","plan_brake_steps")]
                assert t["initial_plan"] == [0,0,0]
                if t["plan_ready_m"] is not None: assert 48.62<=t["plan_ready_m"]<49.0
                failed=next((r for r in rows if r["mode"]=="3"),None)
                if failed:
                    t["first_stop_sample"]={key:float(failed[key]) for key in ("progress_m","speed","compute_ms","scan_source_age_ms","odom_source_age_ms","x","y")}
                t["max_compute_ms"] = max(float(row["compute_ms"]) for row in rows)
                t["max_scan_age_ms"] = max(float(row["scan_source_age_ms"]) for row in rows)
                if t["command_range"]: assert 2.7-1e-5<=t["command_range"][0]<=t["command_range"][1]<=3.0+1e-5
                t["accepted"] = t["accepted"] and not t["timeout"] and not t["stopped"] and not t["collision"]
            clean=[t["lap_times"][:t["failure"][0]-1] if t["failure"] else t["lap_times"] for t in subset]
            per_lap=[[times[i] for times in clean if i<len(times)] for i in range(3)]
            cases[name]=dict(group=group, placement=placement, trials=subset,
                             accepted=sum(t["accepted"] for t in subset), lap_counts=[len(z) for z in per_lap],
                             mean_lap_s=[round(mean(z),3) if z else None for z in per_lap])
    return cases


if __name__=="__main__":
    import sys
    groups = sys.argv[1:] or ["retest-known","retest-new","fresh"]
    data=evaluate(groups)
    prev=json.loads((HERE.parent/"berlin-forward-holdout/results.json").read_text())
    weights=hashlib.sha256((ROOT/"models/avoid-corridor-v2.pt").read_bytes()).hexdigest()
    assert weights==prev["conditions"]["avoid_sha256"]
    out={"conditions":{"map":"Berlin forward","avoid":"avoid-corridor-v2.pt",
        "avoid_sha256":weights,"sensor_limit_ms":150,"deadline_ms":25,"flying_avoid_command_mps":[2.7,3],
        "race":"race-berlin-wsp.pt","laps":3,"repeats":2,"obstacles":3},"cases":data}
    for seed in (int(name.split("-")[-1]) for name in data):
        case=data[f"random-{seed}"]
        tr=HERE/case["group"]/f"random-{seed}-1.csv.corridor.jsonl"
        with tr.open() as stream: frames=[json.loads(l) for l in stream]
        case["first_run_trace"]={"frames":len(frames),"wide_frames":sum(r["wide"] for r in frames),
            "refined_frames":sum(r["subdivisions"]==4 for r in frames),
            "steering_correction_over_0.01_rad":sum(abs(r["policy_steer"]-r["candidate_steer"])>.01 for r in frames)}
    (HERE/"results.json").write_text(json.dumps(out,indent=2)+"\n")
    for name,case in data.items():
        print(name,case["accepted"],case["mean_lap_s"],[t["failure"] for t in case["trials"]])
