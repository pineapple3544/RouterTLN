"""Summarize one directory containing v3-{1,3,5} and v15-{1,3,5} trial logs."""
import argparse
import csv
import json
import re
from pathlib import Path


def read_json(path):
    return json.loads(path.read_text())


def summarize(folder):
    obs=read_json(folder/"observations.json")
    score=read_json(next(folder.glob("race_result_*.json")))
    with (folder/"control.csv").open() as f:
        controls=list(csv.DictReader(f))
    compute=[float(x["compute_ms"]) for x in controls]
    first_fault=next((x for x in controls if x["valid"]=="0"),None)
    log=(folder/"controller.log").read_text() if (folder/"controller.log").exists() else ""
    logged_fault=re.search(r"control_fault=([a-z_]+)",log)
    scenario=folder/"control.csv.scenario.csv"
    laps=score["laps"]
    consecutive=0
    for lap in laps:
        if lap["partial"] or lap["total"]:
            break
        consecutive+=1
    out=dict(stop=obs["reason"],elapsed_s=round(obs["elapsed_s"],3),
             consecutive=consecutive,best_s=score["best_lap"],collisions=score["collisions"],
             teleports=len(score["teleports"]),layouts=len(obs["layouts"]),
             counts=[len(x["boxes"]) for x in obs["layouts"]],
             first_passages=[x["layout"] for x in obs["first_obstacle_station_passages"]],
             completed_lap_times=[lap["time"] for lap in laps],
             compute_over25=sum(x>25 for x in compute),max_compute_ms=max(compute,default=None),
             fault=logged_fault.group(1) if logged_fault else (first_fault["reason"] if first_fault else None),
             final_control_reason=controls[-1]["reason"] if controls else None,
             final_progress_m=float(controls[-1]["progress_m"]) if controls else None)
    if scenario.exists():
        settings=read_json(folder/"settings.json")
        out["residual_checkpoint"]=settings.get("residual_checkpoint","")
        out["longest_measured_low_speed_s"]=obs.get("longest_slow_s")
        predicted=0
        passed_steering=0
        with scenario.open() as f:
            transitions={}
            previous=None
            for row in csv.DictReader(f):
                if abs(float(row["residual_steer"]))>1e-5 or abs(float(row["residual_speed"]))>1e-5:
                    predicted+=1
                    if (row["reason"] in ("scenario_track","scenario_refined_track","scenario_short_track","scenario_swept_track")
                        and abs(float(row["final_steer"])-float(row["base_steer"]))>1e-5
                        and abs(float(row["final_steer"])-float(row["desired_steer"]))<1e-5):
                        passed_steering+=1
                epoch=int(row["epoch"])
                if previous is not None and epoch!=previous:
                    transitions[epoch-1]=float(row["stamp"])
                previous=epoch
        out["nonzero_residual_frames"]=predicted
        out["accepted_residual_steering_frames"]=passed_steering
        lap_events={e["lap"]:e["t"] for e in obs["lap_events"] if e["lap"]>0}
        out["controller_epochs"]=list(transitions)
        out["scorer_laps_seen"]=list(lap_events)
        out["epoch_minus_lap_received_ms"]={
            k:round((stamp-lap_events[k])*1000,1) for k,stamp in transitions.items() if k in lap_events}
        out["epoch_minus_layout_received_ms"]={
            k:round((stamp-obs["layouts"][k]["t"])*1000,1) for k,stamp in transitions.items() if k<len(obs["layouts"])}
    return out


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root",type=Path)
    parser.add_argument("--baseline",type=Path,help="v1.5 trial directory if recorded separately")
    parser.add_argument("--paired-v3",type=Path,help="same v3 binary without the residual")
    args=parser.parse_args()
    if args.baseline and args.paired_v3:
        parser.error("choose one comparison directory")
    root=args.root
    baseline=args.paired_v3 or args.baseline or root
    label="v3-off" if args.paired_v3 else "v15"
    disk_mode="v3" if args.paired_v3 else "v15"
    folders={f"v3-{n}":root/f"v3-{n}" for n in (1,3,5)}
    folders.update({f"{label}-{n}":baseline/f"{disk_mode}-{n}" for n in (1,3,5)})
    result={name:summarize(d) for name,d in folders.items() if (d/"observations.json").exists()}
    common={}
    for count in (1,3,5):
        files=[folders[f"{version}-{count}"]/"observations.json" for version in (label,"v3")]
        if all(f.exists() for f in files):
            a,b=(read_json(f)["layouts"] for f in files)
            common[count]=[x["boxes"]==y["boxes"] for x,y in zip(a,b)]
    print(json.dumps(dict(trials=result,paired_layout_equality=common),indent=2))


if __name__=="__main__":
    main()
