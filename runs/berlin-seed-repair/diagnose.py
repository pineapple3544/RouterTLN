"""Join measured trajectories to their own occupancy snapshots at a matched station."""
import csv
import json
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon
import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "berlin-corridor-diagnosis"))
from snapshots import export
from compare import base, wp, res, x0, y0, W

LENGTH = 48.783921


def load(metrics):
    with metrics.open() as stream:
        rows = list(csv.DictReader(stream))
    trace = metrics.with_name(metrics.name + ".corridor.jsonl")
    frames = [json.loads(line) for line in trace.open()]
    return rows, frames


def station(frame):
    x, y, h = frame["pose"]
    rx, ry, direction = wp[frame["waypoint"]]
    return {"lateral_m": -(x-rx)*math.sin(direction)+(y-ry)*math.cos(direction),
            "yaw_relative_rad": math.remainder(h-direction, 2*math.pi)}


def rectangle(x,y,h,length,width):
    c,s = math.cos(h),math.sin(h)
    unit = np.array([[-1,-1],[1,-1],[1,1],[-1,1]]) / 2
    return unit * [length,width] @ np.array([[c,s],[-s,c]]) + [x,y]


def brake_path(frame,rows):
    # Replay the original guard with the rejected action recorded before the latch.
    obj = frame["step"]
    steer = float(rows[obj]["steer"])
    command = frame["candidate_steer"]
    x,y,h = frame["pose"]
    v = frame["speed"]
    out = []
    grid = base.copy()
    grid.flat[frame["obstacle_cells"]] = True
    dx,dy = np.meshgrid(np.linspace(-.33,.33,7),np.linspace(-.1416,.1416,5))
    for i in range(300):
        dt = .025/3 if i<3 else .01
        if i<3: steer += np.clip(command-steer,-3.2*dt,3.2*dt)
        acc = 0 if i<3 else max(-6.,-v/.15)
        dist = max(0.,v+.5*acc*dt)*dt
        dh = dist*math.tan(steer)/.3302
        x+=dist*math.cos(h+.5*dh); y+=dist*math.sin(h+.5*dh)
        h+=dh;v=max(0.,v+acc*dt)
        out.append([x,y])
        c,s=math.cos(h),math.sin(h)
        ix=np.floor((x+c*dx-s*dy-x0)/res).astype(int)
        iy=np.floor((y+s*dx+c*dy-y0)/res).astype(int)
        hit=grid[iy,ix]
        if hit.any():
            return out,[[float(x0+(col+.5)*res),float(y0+(row+.5)*res),bool(base[row,col])]
                        for col,row in zip(ix[hit],iy[hit])]
        if v<.045:break
    return out,[]


def analyze(seed,reason,files):
    failed = None
    reference = None
    for f in files:
        rows, frames = load(f)
        bad = next((row for row in rows if row["mode"] == "3"), None)
        if bad and bad["reason"] == reason:
            frame = next(x for x in frames if x["step"]==int(bad["step"]))
            if failed is None or frame.get("brake_event"):
                failed = f,rows,frames,frame
        if not bad: reference = f,rows,frames
    if failed is None:raise RuntimeError(f"No recorded {reason} for {seed}")
    filename,rows,frames,frame=failed
    previous=max((x for x in frames if x["step"]<frame["step"]),key=lambda x:x["step"])
    if reference is None:
        ref_file,ref_rows,ref_frames = filename,rows,frames
        station_m = frame["progress"] % LENGTH
    else:
        ref_file,ref_rows,ref_frames = reference
        station_m = frame["progress"]
    good=min(ref_frames,key=lambda x:abs(x["progress"]-station_m))
    assert good["waypoint"]==frame["waypoint"]
    folder=HERE/"snapshots"/str(seed)
    for label,st in (("previous",previous),("failure",frame),("passing",good)):
        export(folder/label,st)
    poses=next(yaml.safe_load_all(filename.with_name(filename.stem+"-poses.yaml").read_text()))["poses"]
    boxes=[(o["position"]["x"],o["position"]["y"],2*math.atan2(o["orientation"]["z"],o["orientation"]["w"])) for o in poses]
    trace,hit=brake_path(frame,rows) if reason=="braking_footprint" else ([],[])
    if frame.get("brake_event"):
        assert hit and np.linalg.norm(np.array(trace[-1])-frame["brake_pose"][:2]) < .02
    result={"failed_run":str(filename.relative_to(HERE)),"passed_section_run":str(ref_file.relative_to(HERE)),
            "failure_progress":frame["progress"],"passing_progress":good["progress"],
            "controller_brake_contact_pose":frame.get("brake_pose"),"rejected_steer":frame.get("candidate_steer"),
            "first_empty":frame["first_empty"],"passed_first_empty":good["first_empty"],
            "failure":station(frame),"passing":station(good),"brake_replay_hit_cells_xy_static":hit}
    center=np.array(frame["pose"][:2])
    panels=[(frame,rows,"Failed approach"),(good,ref_rows,"Passing approach")]
    fixed=HERE/"retest-known"/f"random-{seed}-1.csv"
    if fixed.exists() and (fixed.with_name(fixed.stem+"-run.log")).read_text().find('"iterations"')>=0:
        fixed_rows,fixed_frames=load(fixed)
        matched=min(fixed_frames,key=lambda x:abs(x["progress"]-frame["progress"]))
        if abs(matched["progress"]-frame["progress"])<.12:
            panels.append((matched,fixed_rows,"After correction"))
            export(folder/"after",matched)
            result["after_at_same_station"]=station(matched)
    fig,axs=plt.subplots(1,len(panels),figsize=(4.5*len(panels),4.4),layout="constrained")
    for ax,(st,data,title) in zip(axs,panels):
        ax.imshow(base,extent=(x0,x0+W*res,y0,y0+base.shape[0]*res),origin="lower",cmap="Greys",alpha=.4)
        off=np.array(st["obstacle_cells"])
        ax.scatter(x0+(off%W+.5)*res,y0+(off//W+.5)*res,marker="s",s=7,color="#e18620",label="Occupied cells")
        for bx,by,angle in boxes:
            ax.add_patch(Polygon(rectangle(bx,by,angle,.3,.3),fill=False,edgecolor="#84410b"))
        for group,col,text in ((rows,"#ab2547","Failed run"),(ref_rows,"#007f73","Passing run")):
            target=frame["progress"] if text=="Failed run" else good["progress"]
            seg=[r for r in group if target-3<=float(r["progress_m"])<=target+.012 and r["mode"]!="3"]
            ax.plot([float(r["x"]) for r in seg],[float(r["y"]) for r in seg],color=col,lw=1.9,label=text)
        if title=="After correction":
            seg=[r for r in data if st["progress"]-3<=float(r["progress_m"])<=st["progress"]+.012 and r["mode"]!="3"]
            ax.plot([float(r["x"]) for r in seg],[float(r["y"]) for r in seg],color="#2755bd",lw=2,label="After correction")
            ax.legend(fontsize=7,loc="lower left")
        ax.add_patch(Polygon(rectangle(*st["pose"],.58,.2032),facecolor="#4d6d9c",alpha=.7))
        if title.startswith("Failed") and trace:
            ax.plot(*np.array(trace).T,"k--",lw=1.3,label="Brake replay")
            if hit:ax.plot(hit[0][0],hit[0][1],"kx")
        ax.set(xlim=(center[0]-1.9,center[0]+1.9),ylim=(center[1]-1.7,center[1]+1.7),aspect="equal",xlabel="Map x (m)",title=title)
    axs[0].set_ylabel("Map y (m)")
    axs[0].legend(fontsize=7,loc="lower left")
    fig.suptitle(f"Berlin {seed}   /   {reason}")
    fig.savefig(folder/"comparison.png",dpi=150);plt.close(fig)
    return result


if __name__=="__main__":
    records={}
    for seed,reason in ((39103,"braking_footprint"),(39104,"no_corridor")):
        files=[f for d in sorted(HERE.glob("before*")) if d.is_dir()
               for f in sorted(d.glob(f"random-{seed}-*.csv")) if f.with_name(f.name+".corridor.jsonl").exists()]
        records[seed]=analyze(seed,reason,files)
    (HERE/"diagnosis.json").write_text(json.dumps(records,indent=2)+"\n")
