import csv
import hashlib
import json
import runpy
import sys
from collections import Counter
from pathlib import Path

HERE=Path(__file__).resolve().parent
BASE=HERE.parent/"berlin-sync-braking/offline-v1"
TARGETS=[f"random-{x}" for x in (4135410,6695374,9157059,9285660)]


def records(folder):
    rows={}
    for path in folder.glob("results-*.jsonl"):
        for line in path.read_text().splitlines():
            row=json.loads(line)
            assert row["case"] not in rows
            rows[row["case"]]=row
    return rows


def passed(row):
    return (len(row["laps"])==3 and row["reason"]=="completed" and row["first_rejection"] is None
            and row["contact"] is None and row["min_speed_after_first_half_m"]>=.05
            and all(l["passed"]==3 for l in row["laps"]))


def audit_offline(folder):
    old,new=records(BASE),records(folder)
    assert len(new)==100 and new.keys()==old.keys()
    pairs=[]
    for tag in sorted(old):
        a,b=old[tag],new[tag]
        assert a["spec"]==b["spec"] and a["boxes"]==b["boxes"] and b["branch_entry"] is None
        assert b["spec"]["sim_avoid_min"]==2.7 and b["control_ms"]==25
        if passed(b):
            laps=b["laps"]
            assert laps[0]["recorded_sections"] and laps[2]["brake_steps"]>laps[1]["brake_steps"]>laps[0]["brake_steps"]
        pairs.append(dict(seed=tag,before=passed(a),after=passed(b),reason=(b["first_rejection"] or {}).get("reason", b["reason"]),
                          collision=b["contact"] is not None,laps=len(b["laps"]),min_gap_m=b["min_sat_gap_m"]))
    with (folder/"comparison.csv").open("w",newline="") as stream:
        w=csv.DictWriter(stream,fieldnames=list(pairs[0]));w.writeheader();w.writerows(pairs)
    gains=[r["seed"] for r in pairs if not r["before"] and r["after"]]
    losses=[r["seed"] for r in pairs if r["before"] and not r["after"]]
    return dict(unchanged_pass_records=sum(old[k]==new[k] for k in old if passed(old[k])),
                before=sum(r["before"] for r in pairs),after=sum(r["after"] for r in pairs),gains=gains,losses=losses,
                collisions=sum(r["collision"] for r in pairs),failures=[r for r in pairs if not r["after"]])


def audit_ros(folder):
    summarize=runpy.run_path(str(HERE.parent/"berlin-brake-merge/analyze.py"))["summarize"]
    trials=[]
    for d in sorted(folder.glob("group[01]")):
        for t in summarize(d):
            with (d/(t["tag"]+".csv")).open() as f:rows=list(csv.DictReader(f))
            t["max_compute_ms"]=max(float(z["compute_ms"]) for z in rows)
            t["max_pair_skew_ms"]=max(abs(float(z["scan_source_age_ms"])-float(z["odom_source_age_ms"])) for z in rows)
            t["max_scan_age_ms"]=max(float(z["scan_source_age_ms"]) for z in rows)
            log=(d/(t["tag"]+"-controller.log")).read_text()
            t["sensor_timeout"]=any("timeout" in z["reason"] for z in rows) or "timeout:" in log.lower()
            t["accepted"]=t["accepted"] and not t["stopped"] and not t["collision"] and not t["sensor_timeout"]
            trials.append(t)
    assert len(trials)==8 and {t["tag"].rsplit("-",1)[0] for t in trials}==set(TARGETS)
    return trials


if __name__=="__main__":
    folder=Path(sys.argv[1]).resolve()
    summary=audit_offline(folder)
    if len(sys.argv)>2:summary["ros"]=audit_ros(Path(sys.argv[2]).resolve())
    (folder/"summary.json").write_text(json.dumps(summary,indent=2)+"\n")
    print({k:v for k,v in summary.items() if k not in ("failures","ros")})
    if "ros" in summary:print([(t["tag"],t["accepted"],t["failure"]) for t in summary["ros"]])
