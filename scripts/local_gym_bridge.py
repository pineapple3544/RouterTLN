"""Run the local Gym physics step and its sensor publication as one ordered sample."""
import os
from time import monotonic
from pathlib import Path

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from f1tenth_gym_ros.gym_bridge import GymBridge


class LocalBridge(GymBridge):
    def __init__(self):
        if os.environ.get('EL_SIM_WARMUP') == '1':
            from warm_berlin_sim import warm
            warm(Path(os.environ['EL_MAP_DIR']), os.environ['EL_SIM_CONFIG_DIR'])
        super().__init__()
        self.ready_file = os.environ.get('EL_SIM_READY_FILE')
        noise_seed = os.environ.get('EL_SCAN_NOISE_SEED')
        if noise_seed is not None:
            import numpy as np
            for agent in self.env.unwrapped.sim.agents:
                agent.seed = int(noise_seed)
                agent.scan_rng = np.random.default_rng(agent.seed)
        self.destroy_timer(self.drive_timer)
        self.destroy_timer(self.timer)
        self.acquisition_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(.01, self.sample, clock=self.acquisition_clock)
        self.judge = None
        judge_path = os.environ.get('EL_JUDGE_FILE')
        if judge_path:
            self.judge = open(judge_path, 'w', buffering=65536)
            self.judge.write('stamp,x,y,yaw,speed,collision\n')
        self.trace = None
        path = os.environ.get('EL_BRIDGE_TRACE_FILE')
        if path:
            self.trace = open(path, 'w', buffering=65536)
            self.trace.write('event,start,end,stamp\n')

    def sample(self):
        # Stamp the start, so a slow physics cycle remains visible to the
        # controller freshness check. Both messages describe the same state.
        acquired = self.acquisition_clock.now().to_msg()
        start = monotonic()
        self.drive_timer_callback()
        self.ts = acquired
        updated = monotonic()
        self.timer_callback()
        if self.ready_file:
            Path(self.ready_file).touch()
            self.ready_file = None
        if self.judge:
            stamp = acquired.sec + acquired.nanosec * 1e-9
            kind = self.env.unwrapped.sim.agents[0].collision_kind or 'none'
            if kind == 'none' and self.obs['collisions'][0]:
                kind = 'gym'
            x, y, yaw = self.ego_pose
            self.judge.write(f'{stamp:.9f},{x},{y},{yaw},{self.ego_speed[0]},{kind}\n')
        if self.trace:
            stamp = acquired.sec + acquired.nanosec * 1e-9
            self.trace.write(f'physics,{start:.9f},{updated:.9f},{stamp:.9f}\n'
                             f'publish,{updated:.9f},{monotonic():.9f},{stamp:.9f}\n')


if __name__ == '__main__':
    rclpy.init()
    node = LocalBridge()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        if node.judge:
            node.judge.close()
        if node.trace:
            node.trace.close()
        node.destroy_node()
        rclpy.try_shutdown()
