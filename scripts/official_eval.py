"""Practice with the original GymBridge and race_manager; never patch either."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import select
import shutil
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
BASE = Path("/home/misys/forza_ws/race_stack")


def sha(path):
    with path.open("rb") as stream:
        digest = hashlib.sha256()
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
        return digest.hexdigest()


def cli(*args):
    return subprocess.check_output(["ros2", *args], text=True, timeout=30).strip()


def write_json(path, data):
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n")


def preflight(profile, binary):
    import yaml

    profile, binary = profile.resolve(), binary.resolve()
    params = yaml.safe_load(profile.read_text())["/el_native_controller"]["ros__parameters"]
    if (params.get("official_simulator") is not True or
            params.get("local_steady_sim_clock") is not False or
            params.get("stop_after_laps") != 0 or
            not isinstance(params.get("use_sim_time"), bool) or
            params.get("mode") != "moe" or params.get("unified_checkpoint")):
        raise ValueError("use an explicit official, scorer-controlled, dual-expert profile")
    if not os.access(binary, os.X_OK):
        raise FileNotFoundError(f"controller not built or executable: {binary}")
    root = Path(params["map_dir"]).resolve()
    if root.name != "berlin":
        raise ValueError("official evaluation is Berlin only")
    stem = root / root.name
    if Path(str(stem) + "_orig.yaml").is_file():
        stem = Path(str(stem) + "_orig")
    map_yaml = Path(str(stem) + ".yaml")
    map_image = (root / yaml.safe_load(map_yaml.read_text())["image"]).resolve()
    waypoints = root / "global_waypoints.json"
    waypoint = json.loads(waypoints.read_text())["global_traj_wpnts_iqp"]["wpnts"][0]
    # Only the simulator launch receives its start pose. The controller
    # initializes globally from LiDAR and the permitted static map.
    sim_yaml = BASE / "stack_master/config/SIM/sim.yaml"
    physics_yaml = BASE / "stack_master/config/SIM/sim_params.yaml"
    packages = {}
    for package, executables in (("f1tenth_gym_ros", {"gym_bridge"}),
                                 ("race_manager", {"obstacle_spawner", "scorer"})):
        available = {line.split()[-1] for line in cli("pkg", "executables", package).splitlines() if line.strip()}
        if not executables <= available:
            raise RuntimeError(f"{package}: missing {sorted(executables-available)}")
        packages[package] = cli("pkg", "prefix", "--share", package)
    manager_yaml = Path(packages["race_manager"]) / "config/race_manager_params.yaml"
    inputs = [profile, binary, map_yaml, map_image, waypoints, root / "router.yaml",
              Path(params["race_checkpoint"]), Path(params["avoid_checkpoint"]),
              sim_yaml, physics_yaml, manager_yaml]
    fingerprints = {str(path): sha(path) for path in inputs}
    clock = str(params["use_sim_time"]).lower()
    bridge = ["ros2", "run", "f1tenth_gym_ros", "gym_bridge", "--ros-args",
              "-r", "__node:=bridge", "--params-file", str(sim_yaml),
              "-p", "map_path:="+str(map_yaml), "-p", "sim_params:="+str(physics_yaml),
              "-p", "kb_teleop:=false", "-p", "use_sim_time:="+clock,
              "-p", "sx:="+str(waypoint["x_m"]), "-p", "sy:="+str(waypoint["y_m"]),
              "-p", "stheta:="+str(waypoint["psi_rad"])]
    manager_args = ["--ros-args", "--params-file", str(manager_yaml),
                    "-p", "waypoints_file:="+str(waypoints), "-p", "use_sim_time:="+clock]
    return dict(params=params, binary=str(binary), fingerprints=fingerprints,
                packages=packages, bridge=bridge, manager_args=manager_args)


def controller_cmd(setup, frozen, output):
    # Only the metrics destination changes across runs.
    return [setup["binary"], "--ros-args", "--params-file", str(frozen),
            "-p", "metrics:="+str(output / "control.csv")]


def check_frozen(setup, frozen, expected):
    if sha(frozen) != expected or any(sha(Path(p)) != h for p, h in setup["fingerprints"].items()):
        raise RuntimeError("profile, controller, map, model or package configuration changed")


def start(children, name, command, output):
    with (output / (name + ".log")).open("w") as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
    children[name] = process
    return process


def require_alive(children, names):
    for name in names:
        if children[name].poll() is not None:
            raise RuntimeError(f"{name} exited ({children[name].returncode}); inspect its log")


def terminate(children, names):
    processes = [children[name] for name in names if name in children]
    for sig, timeout in ((signal.SIGINT, 8), (signal.SIGTERM, 3), (signal.SIGKILL, 2)):
        for process in processes:
            if process.poll() is None:
                try:
                    os.killpg(process.pid, sig)
                except ProcessLookupError:
                    pass
        deadline = time.monotonic() + timeout
        while any(p.poll() is None for p in processes) and time.monotonic() < deadline:
            time.sleep(.05)
        if all(p.poll() is not None for p in processes):
            break


def run_trial(setup, frozen, expected, output, count, seed, duration):
    # A separate, passive orchestration node receives only scan headers,
    # obstacle-publication presence and whether a positive drive command began.
    # It never reads obstacle coordinates, publishes drive, or passes data to TLN.
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import LaserScan
    from geometry_msgs.msg import PoseArray
    from ackermann_msgs.msg import AckermannDriveStamped

    check_frozen(setup, frozen, expected)
    output.mkdir()
    result_dir = output / "race_manager"
    result_dir.mkdir()
    seed_arg = ["-p", f"seed:={seed}"]
    manager = setup["manager_args"]
    commands = {
        "bridge": setup["bridge"],
        "scorer": ["ros2", "run", "race_manager", "scorer", *manager, *seed_arg,
                   "-p", "map_name:=berlin", "-p", "log_dir:="+str(result_dir)],
        "spawner": ["ros2", "run", "race_manager", "obstacle_spawner", *manager, *seed_arg,
                    "-p", f"num_obstacles:={count}"],
        "controller": controller_cmd(setup, frozen, output),
    }
    write_json(output / "commands.json", commands)
    children = {}
    outcome = {"obstacles": count, "seed": seed, "requested_wall_seconds": duration,
               "profile_sha256": expected, "stop_reason": "setup_error",
               "official_lap_cap_verified": False}
    node = None
    active_since = None
    scan_stamp = None
    scan_received = None
    obstacle_seen = False
    try:
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        node = rclpy.create_node("routertln_eval_observer",
            parameter_overrides=[Parameter("use_sim_time", value=setup["params"]["use_sim_time"])])
        def scan(msg):
            nonlocal scan_stamp, scan_received
            scan_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            scan_received = time.monotonic()
        def obstacle_event(_msg):
            nonlocal obstacle_seen
            obstacle_seen = True
        def drive(msg):
            nonlocal active_since
            if active_since is None and msg.drive.speed > 0:
                active_since = time.monotonic()
        subscriptions = [
            node.create_subscription(LaserScan, "/scan", scan, qos_profile_sensor_data),
            node.create_subscription(PoseArray, "/sim/obstacles", obstacle_event, qos_profile_sensor_data),
            node.create_subscription(AckermannDriveStamped, "/drive", drive, qos_profile_sensor_data),
        ]
        def spin():
            rclpy.spin_once(node, timeout_sec=.1)
        def fresh_scan():
            if scan_stamp is None or scan_stamp <= 0 or time.monotonic() - scan_received > .15:
                return False
            now = node.get_clock().now().nanoseconds * 1e-9
            return now > 0 and -.05 <= now-scan_stamp <= .15
        def wait_for(predicate, seconds, names, message):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                require_alive(children, names)
                spin()
                if predicate():
                    return
            raise RuntimeError(message)
        discovery = time.monotonic() + 2
        while time.monotonic() < discovery:
            spin()
        for topic in ("/scan", "/drive", "/sim/obstacles"):
            if node.count_publishers(topic):
                raise RuntimeError(f"practice domain already has a publisher on {topic}; choose an unused domain")
        start(children, "bridge", commands["bridge"], output)
        wait_for(fresh_scan, 40, ("bridge",),
                 "no fresh original /scan; inspect bridge log and ROS clock setting")
        start(children, "scorer", commands["scorer"], output)
        start(children, "spawner", commands["spawner"], output)
        wait_for(lambda: obstacle_seen and fresh_scan() and node.count_publishers("/sim/obstacles")==1,
                 30, ("bridge", "scorer", "spawner"),
                 "official spawner did not publish obstacles or scan clock is incompatible")
        if node.count_publishers("/drive") or node.count_publishers("/scan") != 1:
            raise RuntimeError("unexpected drive or scan publisher")
        start(children, "controller", commands["controller"], output)
        wait_for(lambda: active_since is not None, 40,
                 ("bridge", "scorer", "spawner", "controller"),
                 "no positive drive command; inspect controller log")
        print(f"obs{count}: first positive command; follow official scorer log in {output}.", flush=True)
        print("Press Enter when the official scorer reaches 10 laps or declares a terminal conflict.",
              flush=True)
        while time.monotonic() - active_since < duration:
            spin()
            require_alive(children, ("bridge", "spawner", "controller"))
            if children["scorer"].poll() is not None:
                if children["scorer"].returncode != 0:
                    raise RuntimeError("official scorer failed; inspect scorer.log")
                outcome["stop_reason"] = "official_scorer_exit"
                break
            if sys.stdin.isatty() and select.select([sys.stdin], [], [], 0)[0]:
                sys.stdin.readline()
                outcome["stop_reason"] = "operator_observed_official_finish"
                break
        else:
            outcome["stop_reason"] = "practice_wall_time_limit"
        outcome["elapsed_wall_seconds_since_first_positive_command"] = time.monotonic() - active_since
    except BaseException as error:
        outcome["error"] = str(error) or type(error).__name__
        outcome["stop_reason"] = "interrupted" if isinstance(error, KeyboardInterrupt) else "error"
        raise
    finally:
        # Finalize the original scorer while its simulator is still available.
        terminate(children, ("scorer", "controller"))
        terminate(children, ("spawner", "bridge"))
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        outcome["process_exit_codes"] = {name: proc.poll() for name, proc in children.items()}
        outcome["official_result_files"] = [str(p.relative_to(output)) for p in sorted(result_dir.rglob("*")) if p.is_file()]
        write_json(output / "trial.json", outcome)
    if not outcome["official_result_files"]:
        raise RuntimeError(f"original scorer wrote no results: {output}")
    return outcome


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=["check", "run", "suite", "controller"])
    parser.add_argument("--profile", type=Path, default=ROOT / "config/official/v1.yaml")
    parser.add_argument("--binary", type=Path, default=Path("/build/el_controller"))
    parser.add_argument("--output", type=Path)
    parser.add_argument("--count", type=int, choices=[1, 3, 5])
    parser.add_argument("--seed", type=int)
    parser.add_argument("--duration", type=float, default=180.)
    parser.add_argument("--domain", type=int, help="isolated practice domain; default 74 for run/suite")
    args = parser.parse_args()
    if not 0 < args.duration <= 180:
        parser.error("practice time limit must be in (0, 180]")
    if args.action in ("run", "suite") and (args.seed is None or args.seed < 0):
        parser.error("supply a nonnegative practice seed")
    if args.action == "run" and args.count is None:
        parser.error("run requires --count 1, 3 or 5")
    if args.action != "run" and args.count is not None:
        parser.error("--count applies only to run")
    if args.action != "check" and args.output is None:
        parser.error("provide a new --output directory")
    if args.domain is not None and not 0 <= args.domain <= 232:
        parser.error("ROS domain must be 0..232")
    if args.domain is not None or args.action in ("run", "suite"):
        os.environ["ROS_DOMAIN_ID"] = str(args.domain if args.domain is not None else 74)
    setup = preflight(args.profile, args.binary)
    if args.action == "check":
        print(json.dumps(setup, indent=2))
        return
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    frozen = output / "controller.yaml"
    shutil.copyfile(args.profile, frozen)
    expected = setup["fingerprints"][str(args.profile.resolve())]
    check_frozen(setup, frozen, expected)
    write_json(output / "manifest.json", dict(setup, profile_sha256=expected,
        ros_domain=os.environ.get("ROS_DOMAIN_ID", "0"), action=args.action))
    if args.action == "controller":
        # For the instructor's already-running simulator / race manager.
        # No simulator, scorer or spawner is started by this action.
        command = controller_cmd(setup, frozen, output)
        write_json(output / "commands.json", {"controller": command})
        os.execv(command[0], command)
    counts = [1, 3, 5] if args.action == "suite" else [args.count]
    outcomes = []
    for count in counts:
        try:
            result = run_trial(setup, frozen, expected, output / f"obs{count}",
                               count, args.seed, args.duration)
            outcomes.append(result)
        except Exception as error:
            trial = output / f"obs{count}" / "trial.json"
            outcomes.append(json.loads(trial.read_text()) if trial.exists() else
                            {"obstacles": count, "error": str(error)})
            print(f"obs{count} failed: {error}", file=sys.stderr, flush=True)
            # Setup/transport failure is recorded, never silently retried.
        finally:
            write_json(output / "suite.json", outcomes)
    if any("error" in row or not row.get("official_result_files") for row in outcomes):
        sys.exit(1)


if __name__ == "__main__":
    main()
