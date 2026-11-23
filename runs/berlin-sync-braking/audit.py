"""Compare the same frozen cases and audit cold-start ROS safety and sensor timing."""
import csv
import hashlib
import json
import runpy
from collections import Counter
from pathlib import Path

HERE=Path(__file__).resolve().parent
BASE=HERE.parent/"berlin-forward-holdout-100"


def load(path):
    return json.loads(path.read_text())


def collect(folder):
    rows={}
    for path in folder.glob("results-*.jsonl"):
        for line in path.read_text().splitlines():
            row=json.loads(line)
            assert row["case"] not in rows
            rows[row["case"]]=row
    assert len(rows)==100
    return rows


def clean(r):
    return (r["reason"]=="completed" and len(r["laps"])==3 and not r["contact"] and not r["first_rejection"]
            and r["min_speed_after_first_half_m"]>=.05 and all(l["passed"]==3 for l in r["laps"]))


def rejection(r):
    return (r["first_rejection"] or {}).get("reason") or r["reason"]


def audit(version):
    offline=HERE/("offline-"+version)
    new=collect(offline)
    old=collect(BASE/"offline")
    assert new.keys()==old.keys()
    counts=Counter()
    changes=[]
    for tag in sorted(new):
        r,b=new[tag],old[tag]
        assert r["spec"]==b["spec"] and r["boxes"]==b["boxes"] and r["branch_entry"] is None
        assert r["control_ms"]==25 and r["spec"]["sim_avoid_min"]==2.7
        if clean(r):
            laps=r["laps"]
            assert laps[0]["recorded_sections"]>0 and laps[2]["brake_steps"]>laps[1]["brake_steps"]>laps[0]["brake_steps"]
        counts[(clean(b),clean(r))]+=1
        changes.append(dict(layout=tag,previous=clean(b),current=clean(r),previous_reason=rejection(b),
                            current_reason=rejection(r),collision=r["contact"] is not None,laps=len(r["laps"]),gap_m=r["min_sat_gap_m"]))
    with (HERE/("comparison-"+version+".csv")).open("w",newline="") as stream:
        w=csv.DictWriter(stream,changes[0].keys());w.writeheader();w.writerows(changes)
    summarize=runpy.run_path(str(HERE.parent/"berlin-brake-merge/analyze.py"))["summarize"]
    trials=[]
    for folder in sorted((HERE/("ros-"+version)).glob("random-*")):
        if not folder.is_dir(): continue
        rr=summarize(folder)
        assert len(rr)==2
        for t in rr:
            p=folder/(t["tag"]+".csv")
            with p.open() as stream: metrics=list(csv.DictReader(stream))
            t["max_pair_skew_ms"]=max(abs(float(r["scan_source_age_ms"])-float(r["odom_source_age_ms"])) for r in metrics)
            t["max_scan_age_ms"]=max(float(r["scan_source_age_ms"]) for r in metrics)
            t["max_compute_ms"]=max(float(r["compute_ms"]) for r in metrics)
            log=(folder/(t["tag"]+"-controller.log")).read_text()
            t["sensor_timeout"]=any("timeout" in r["reason"] for r in metrics) or "timeout:" in log.lower()
            t["accepted"]=all([t["accepted"],not t["stopped"],not t["collision"],not t["sensor_timeout"]])
            assert t["max_pair_skew_ms"]<1e-5
            trials.append(t)
    assert len(trials)==4
    result={"previous_pass":sum(clean(r) for r in old.values()),"current_pass":sum(clean(r) for r in new.values()),
            "new_pass":[c["layout"] for c in changes if c["current"] and not c["previous"]],
            "new_fail":[c["layout"] for c in changes if c["previous"] and not c["current"]],
            "collisions":sum(r["contact"] is not None for r in new.values()),
            "reasons":dict(Counter(rejection(r) for r in new.values())),"ros":trials}
    (HERE/("summary-"+version+".json")).write_text(json.dumps(result,indent=2)+"\n")
    print({k:v for k,v in result.items() if k!="ros"})
    print([(t["tag"],t["accepted"],t["lap_times"],t["failure"]) for t in trials])


if __name__=="__main__":
    import sys
    audit(sys.argv[1])
