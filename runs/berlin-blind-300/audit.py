from pathlib import Path
import csv,json,runpy,statistics
from collections import Counter
from scipy.stats import beta
HERE=Path(__file__).resolve().parent
TARGETS=json.loads((HERE/"preregister.json").read_text())["ros_selected"]
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
    expected={f"{seed}-{repeat}" for seed in TARGETS for repeat in (1,2)}
    assert len(trials)==len(expected) and {t["tag"] for t in trials}==expected
    return trials



if __name__=="__main__":
    layouts=runpy.run_path(str(HERE/"offline_audit.py"))["main"]()
    trials=audit_ros(HERE/"ros")
    faults=[r for r in layouts if not r["accepted"]]
    bad=[t for t in trials if not t["accepted"]]
    double_clean=sum(all(t["accepted"] for t in trials if t["tag"].rsplit("-",1)[0]==k) for k in TARGETS)
    def upper(failed,n):
        return 1 if failed==n else beta.ppf(.95,failed+1,n-failed)
    summary=dict(offline=len(layouts),offline_clean=len(layouts)-len(faults),offline_failures=faults,
                 offline_upper95=upper(len(faults),len(layouts)),offline_collisions=sum(r["collision"] for r in layouts),
                 ros_trials=len(trials),ros_clean=len(trials)-len(bad),ros_both=double_clean,
                 ros_layout_pair_upper95=upper(len(TARGETS)-double_clean,len(TARGETS)),
                 ros_collisions=sum(t["collision"] for t in trials),ros_stops=sum(t["stopped"] for t in trials),
                 ros_timeouts=sum(t["sensor_timeout"] for t in trials),ros_failures=bad,
                 max_compute_ms=max(t["max_compute_ms"] for t in trials),max_scan_age_ms=max(t["max_scan_age_ms"] for t in trials))
    (HERE/"ros-results.json").write_text(json.dumps(trials,indent=2)+"\n")
    (HERE/"summary.json").write_text(json.dumps(summary,indent=2)+"\n")
    print({k:v for k,v in summary.items() if "failures" not in k})
