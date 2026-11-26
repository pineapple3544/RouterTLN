"""Run inside a sourced ROS Humble race_stack container with a CUDA controller build."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32, String
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.qos import qos_profile_sensor_data
from race_manager.track import Track


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=["v15", "v3"])
    parser.add_argument("count", type=int, choices=[1, 3, 5])
    parser.add_argument("output", type=Path)
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument("--full-duration", action="store_true", help="continue after lap 10 through 180 seconds")
    parser.add_argument("--residual-checkpoint", type=Path, help="RCN1 or RCN2 weights for the v3 tracker")
    parser.add_argument("--gate-capture", action="store_true", help="record sensor features for offline calibration")
    args = parser.parse_args()
    if args.residual_checkpoint and (args.mode != "v3" or not args.residual_checkpoint.is_file()):
        parser.error("residual-checkpoint requires v3 and a readable weight file")
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    wp = "/workspace/maps/berlin/global_waypoints.json"
    cfg = str(Path(get_package_share_directory("race_manager")) / "config/race_manager_params.yaml")
    track = Track(wp)
    processes = []
    def launch(name, cmd):
        with (out / (name+".log")).open("w") as log:
            proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append((name,proc))

    rclpy.init()
    node = rclpy.create_node("external_trial_observer")
    layouts, laps, passages = [], [], []
    poses=[]
    odom = last_layout = first_s = prev_s = first_drive = last_motion = None
    scan_seen = False
    last_speed = 0.
    slow_since = None
    longest_slow_s = 0.
    hit = None
    def layout(msg):
        nonlocal last_layout, first_s
        boxes = [(p.position.x,p.position.y,p.orientation.z,p.orientation.w) for p in msg.poses]
        if boxes == last_layout:
            return
        now = time.time()
        layouts.append(dict(t=now,boxes=boxes))
        last_layout = boxes
        start_s = prev_s if prev_s is not None else 0.
        if boxes:
            stations = [track.nearest_s(x,y) for x,y,_,_ in boxes]
            first_s = min(stations, key=lambda s:(s-start_s)%track.length)

    def pose(msg):
        nonlocal odom, prev_s, first_s, last_motion, last_speed, slow_since, longest_slow_s
        p = msg.pose.pose.position
        odom = (p.x,p.y)
        t=time.time()
        if not poses or t-poses[-1][0]>=.05:
            poses.append((t,*odom))
        s = track.nearest_s(*odom)
        if prev_s is not None and first_s is not None:
            delta = track.arc_delta(prev_s,s)
            if 0 < delta < 1.5 and 0 < (first_s-prev_s)%track.length <= delta+1e-5:
                passages.append(dict(layout=len(layouts)-1,t=time.time(),station=first_s))
                first_s = None
        prev_s = s
        last_speed = abs(msg.twist.twist.linear.x)
        if last_speed > .12:
            last_motion = time.monotonic()
            slow_since = None
        elif last_motion is not None and first_drive is not None:
            now = time.monotonic()
            if slow_since is None:
                slow_since = now
            longest_slow_s = max(longest_slow_s, now-slow_since)
    def drive(msg):
        nonlocal first_drive
        if msg.drive.speed > .05 and first_drive is None:
            first_drive=time.monotonic()
    def collision(msg):
        nonlocal hit
        if msg.data in ("obstacle","wall"):
            hit=msg.data
    def scan(msg):
        nonlocal scan_seen
        scan_seen=True
    node.create_subscription(PoseArray,"/sim/obstacles",layout,10)
    node.create_subscription(Odometry,"/car_state/odom_GT",pose,10)
    node.create_subscription(Int32,"/sim/lap_count",lambda m:laps.append(dict(lap=m.data,t=time.time())),10)
    node.create_subscription(String,"/sim/collision",collision,10)
    node.create_subscription(AckermannDriveStamped,"/drive",drive,10)
    node.create_subscription(LaserScan,"/scan",scan,qos_profile_sensor_data)
    def spin():
        rclpy.spin_once(node,timeout_sec=.05)
        for name,proc in processes:
            if proc.poll() is not None:
                raise RuntimeError(f"{name} exited {proc.returncode}")
    base=["--ros-args","--params-file",cfg,"-p",f"waypoints_file:={wp}","-p",f"seed:={args.seed}"]
    try:
        launch("bridge",["bash","/workspace/scripts/run_sim.sh"])
        launch("spawner",["ros2","run","race_manager","obstacle_spawner",*base,"-p",f"num_obstacles:={args.count}"])
        launch("scorer",["ros2","run","race_manager","scorer",*base,"-p","map_name:=berlin","-p",f"log_dir:={out}"])
        start=time.monotonic()
        while not (scan_seen and odom and last_layout and len(last_layout)==args.count):
            spin()
            if time.monotonic()-start>90:
                raise RuntimeError(f"setup timeout: scan={scan_seen} odom={odom} layout={last_layout}")
        params = dict(mode="scenario" if args.mode=="v3" else "moe",inference_device="cpu",
                      max_speed=3.5,deadline_ms=25.0,map_dir="/workspace/maps/berlin",
                      race_checkpoint="/workspace/models/v1.5/race.pt",metrics=str(out/"control.csv"))
        if args.mode=="v3":
            params["residual_checkpoint"]=str(args.residual_checkpoint.resolve()) if args.residual_checkpoint else ""
            params["scenario_gpu_profile"]=str(out/"gpu.csv")
            if args.gate_capture:
                params["scenario_gate_capture"]=str(out/"features.jsonl")
            params["scenario_no_path_snapshot"]=str(out/"no_path.json")
        if args.mode=="v15":
            params.update(official_simulator=True,avoid_checkpoint="/workspace/models/v1.5/avoid.pt",
                          lap_planning=True,sim_section_speed_profile=True,sim_race_only=True,
                          sim_avoid_min=2.7,sim_avoid_max=3.5,straight_boost=True,
                          avoid_corridor_steer_seed=True,avoid_policy_first=True,
                          shared_speed_band=True,whole_lap_path=True,avoid_exit_pace=True,
                          cluster_entry=True,entry_candidates="both")
        cmd=["/build/el_controller","--ros-args"]
        for k,v in params.items():
            if v=="":  # ROS CLI requires explicit quotes to denote an empty string.
                v="''"
            cmd.extend(["-p",f"{k}:={str(v).lower() if isinstance(v,bool) else v}"])
        (out/"settings.json").write_text(json.dumps(params,indent=2))
        artifacts = {"/build/el_controller": hashlib.sha256(Path(cmd[0]).read_bytes()).hexdigest()}
        if args.residual_checkpoint:
            artifacts[str(args.residual_checkpoint)] = hashlib.sha256(args.residual_checkpoint.read_bytes()).hexdigest()
        (out / "artifacts.json").write_text(json.dumps(dict(seed=args.seed, obstacle_count=args.count,
            files=artifacts), indent=2))
        launch("controller",cmd)
        started=time.monotonic()
        while True:
            spin()
            now=time.monotonic()
            if hit:
                reason=hit
                break
            if not args.full_duration and laps and laps[-1]["lap"]>=10:
                reason="ten_laps"
                break
            if first_drive is not None:
                if now-first_drive>=180:
                    reason="duration"
                    break
                if now-max(last_motion or first_drive,first_drive)>6:
                    reason="stopped"
                    break
            elif now-started>35:
                reason="no_start"
                break
        for _ in range(20):
            rclpy.spin_once(node,timeout_sec=.01)
        (out/"observations.json").write_text(json.dumps(dict(reason=reason,first_drive_monotonic_s=first_drive,end_monotonic_s=now,elapsed_s=now-(first_drive or started),
            longest_slow_s=longest_slow_s,full_duration=args.full_duration,
            poses=poses,layouts=layouts,lap_events=laps,first_obstacle_station_passages=passages,final_speed=last_speed),indent=2))
        print(out.name,reason,len(laps),len(layouts),flush=True)
    finally:
        for _,proc in reversed(processes):
            if proc.poll() is None:
                os.killpg(proc.pid,signal.SIGINT)
        for _,proc in reversed(processes):
            try:
                proc.wait(timeout=6)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid,signal.SIGKILL)
                proc.wait()
        node.destroy_node()
        rclpy.shutdown()


if __name__=="__main__":
    main()
