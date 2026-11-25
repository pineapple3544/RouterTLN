"""Compile the simulator's motion, scan and obstacle JIT paths before ROS starts."""
from pathlib import Path
import json
import math

import numpy as np
import yaml
from f110_gym.envs.base_classes import RaceCar
from f110_gym.envs.f110_env import F110Env


def warm(map_dir, config_dir):
    track = json.loads((map_dir / "global_waypoints.json").read_text())
    start = track["global_traj_wpnts_iqp"]["wpnts"][0]
    x, y, yaw = (start[key] for key in ("x_m", "y_m", "psi_rad"))
    params = yaml.safe_load((Path(config_dir) / "sim_params.yaml").read_text())
    shared = ("scan_simulator", "scan_angles", "cosines", "side_distances", "static_obstacles")
    saved = {key: getattr(RaceCar, key) for key in shared}
    try:
        env = F110Env(map=str(map_dir / map_dir.name), map_ext=".png", params=params, num_agents=1)
        env.reset(poses=np.array([[x, y, yaw]]))
        command = np.array([[0.01, 1.0]])
        env.step(command)
        RaceCar.static_obstacles = [(x + 1.5 * math.cos(yaw), y + 1.5 * math.sin(yaw), yaw, .30, .30)]
        env.step(command)
    finally:
        for key, value in saved.items():
            setattr(RaceCar, key, value)
