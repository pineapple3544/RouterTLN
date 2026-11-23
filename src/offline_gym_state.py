"""Data-only snapshot of the single-agent F110 Gym's mutable physics state."""
import hashlib
import json
import numpy as np

ENV = ("poses_x", "poses_y", "poses_theta", "collisions", "near_start", "num_toggles",
       "lap_times", "lap_counts", "current_time", "near_starts", "toggle_list",
       "start_xs", "start_ys", "start_thetas", "start_rot")
SIM = ("agent_poses", "collisions", "collision_idx")
CAR = ("state", "opp_poses", "accel", "steer_angle_vel", "steer_buffer",
       "in_collision", "collision_kind", "static_beams")


def pack(value):
    if isinstance(value, np.ndarray):
        return {"dtype": value.dtype.str, "values": value.tolist(), "shape": value.shape}
    if isinstance(value, np.generic):
        return value.item()
    return value


def capture(env):
    car = env.sim.agents[0]
    groups = [(env, ENV), (env.sim, SIM), (car, CAR)]
    data = [{name: pack(getattr(obj, name)) for name in fields} for obj, fields in groups]
    return json.dumps([data, car.scan_rng.bit_generator.state], default=pack)


def restore(env, payload):
    groups, rng = json.loads(payload)
    car = env.sim.agents[0]
    for (obj, fields), data in zip([(env, ENV), (env.sim, SIM), (car, CAR)], groups):
        if set(data) != set(fields):
            raise ValueError("incompatible Gym snapshot")
        for name in fields:
            val = data[name]
            if isinstance(val, dict):
                val = np.asarray(val["values"], dtype=val["dtype"]).reshape(val["shape"])
            setattr(obj, name, val)
    car.scan_rng.bit_generator.state = rng


def displace(env, dx, dy, heading, speed):
    car = env.sim.agents[0]
    car.state[[0, 1, 4]] += [dx, dy, heading]
    car.state[3] = max(0., car.state[3] + speed)
    env.sim.agent_poses[0] = car.state[[0, 1, 4]]


def fingerprint(paths):
    result = hashlib.sha256()
    for path in json.loads(paths):
        with open(path, "rb") as stream:
            digest = hashlib.sha256()
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
            result.update(digest.digest())
    return result.hexdigest()
