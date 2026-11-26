#!/usr/bin/env python3
"""Generate the fixed long-run test layouts A-E for Berlin (config/longrun_layouts.json).

Test-scenario only: the controller never reads this file. Obstacle indices are picked by
scoring waypoint features (curvature via the same formula as el::route_curvature, raceline
speed/braking, line-of-sight from the static map) and then checked for passability against
the static map. el_scenario places obstacle i at waypoint (index + i*stride) % N with lateral
offset offsets[i] (left of the raceline is positive) and the waypoint yaw, so every layout is an
arithmetic progression of indices; 1 and 3 obstacles are prefixes of the 5-obstacle list.

Run inside the sim container:  python3 tools/longrun_layouts.py [--check-only]
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image
import yaml

ROOT = Path(__file__).resolve().parents[1]
MAP = ROOT / 'maps/berlin'
OUT = ROOT / 'config/longrun_layouts.json'

START_INDEX = 37            # validated start pose used by all Berlin dev runs
OBSTACLE_SIZE = 0.30        # bridge default cube side; el_scenario aligns it with the waypoint yaw
CAR_WIDTH = 0.31            # race_manager car_width
MARGIN = 0.05               # per side, race_manager squeeze_clearance -> 0.41 m passable width
PASS_WIDTH = CAR_WIDTH + 2 * MARGIN
EXTRA_MARGIN = 0.10         # our check is stricter than race_manager: needs PASS_WIDTH + this
MIN_START_GAP = 3.0         # metres from the layout's start pose, both directions (el_scenario
                            # counts laps from the start pose, not from the s=0 line)
MIN_EXISTING_GAP = 0.5      # a layout "repeats" a dev layout if every obstacle is within this
SWITCH_S = 1.0              # two obstacles closer than this along s must share a free side

LAYOUTS = {
    'A': dict(seed=3101, character='spread evenly around the lap', strides=range(90, 104)),
    'B': dict(seed=3102, character='clustered in consecutive corners / a chicane', strides=range(12, 26)),
    'C': dict(seed=3103, character='right after blind corners', strides=range(35, 110)),
    'D': dict(seed=3104, character='alternating sides back-to-back', strides=range(15, 31)),
    # E starts in the s~12 m acceleration zone: with the shared start (s=3.7 m) the 3 m start gap
    # would exclude the main straight and its braking zone, which is the point of this layout.
    'E': dict(seed=3105, character='fastest straights and their braking zones', strides=range(40, 120),
              start=120),
}


def load_track():
    w = json.loads((MAP / 'global_waypoints.json').read_text())['global_traj_wpnts_iqp']['wpnts']
    pts = np.array([(p['x_m'], p['y_m'], p['psi_rad']) for p in w], float)
    # orient_track(): drop a duplicated closing point
    if math.hypot(*(pts[0, :2] - pts[-1, :2])) < .05:
        pts, w = pts[:-1], w[:-1]
    seg = np.linalg.norm(np.diff(pts[:, :2], axis=0), axis=1)
    s = np.r_[0., np.cumsum(seg)]
    length = s[-1] + np.linalg.norm(pts[0, :2] - pts[-1, :2])
    vx = np.array([p['vx_mps'] for p in w[:len(pts)]])
    ax = np.array([p['ax_mps2'] for p in w[:len(pts)]])
    return pts, s, length, vx, ax


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def route_curvature(pts, i):
    """Same as el::route_curvature in include/el/scenarios.hpp."""
    n = len(pts)
    a, b = pts[(i - 5) % n], pts[(i + 5) % n]
    length = sum(math.hypot(*(pts[(i + j + 1) % n, :2] - pts[(i + j) % n, :2])) for j in range(-5, 5))
    return wrap(b[2] - a[2]) / max(length, .05)


def road_region(k):
    k = abs(k)
    return 'straight' if k < .1 else 'transition' if k < .25 else 'corner'


class Grid:
    def __init__(self):
        meta = yaml.safe_load((MAP / 'berlin.yaml').read_text())
        img = np.asarray(Image.open(MAP / meta['image']).convert('L'), float) / 255.
        occ = 1. - img if not meta.get('negate', 0) else img
        self.occupied = occ > meta['free_thresh']   # conservative: anything not clearly free
        self.res = meta['resolution']
        self.ox, self.oy = meta['origin'][:2]
        self.h = self.occupied.shape[0]

    def blocked(self, x, y):
        c = int((x - self.ox) / self.res)
        r = self.h - 1 - int((y - self.oy) / self.res)
        if not (0 <= r < self.h and 0 <= c < self.occupied.shape[1]):
            return True
        return bool(self.occupied[r, c])

    def ray(self, x, y, dx, dy, limit=4.0, step=0.01):
        d = 0.
        while d < limit:
            if self.blocked(x + dx * d, y + dy * d):
                return d
            d += step
        return limit

    def clear(self, a, b, step=0.02):
        n = max(2, int(math.hypot(b[0] - a[0], b[1] - a[1]) / step))
        return all(not self.blocked(a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
                   for t in np.linspace(0, 1, n))


def obstacle_xy(pts, i, d):
    x, y, yaw = pts[i]
    return x - math.sin(yaw) * d, y + math.cos(yaw) * d


def side_gaps(grid, pts, i, d):
    """Free lateral width left/right of an obstacle at waypoint i, offset d (yaw-aligned cube)."""
    x, y, yaw = pts[i]
    ox, oy = obstacle_xy(pts, i, d)
    nx, ny = -math.sin(yaw), math.cos(yaw)
    half = OBSTACLE_SIZE / 2
    # sample the three lateral lines through the front, centre and back of the cube
    left = right = 9.
    for t in (-half, 0., half):
        cx, cy = ox + math.cos(yaw) * t, oy + math.sin(yaw) * t
        left = min(left, grid.ray(cx, cy, nx, ny) - half)
        right = min(right, grid.ray(cx, cy, -nx, -ny) - half)
    return left, right


def visibility(grid, pts, s, length, i, d, horizon=8.0):
    """Metres of raceline before the obstacle from which its centre is in line of sight."""
    target = obstacle_xy(pts, i, d)
    n = len(pts)
    seen = 0.
    for k in range(1, n):
        j = (i - k) % n
        back = (s[i] - s[j]) % length
        if back > horizon:
            break
        if not grid.clear(pts[j, :2], target):
            break
        seen = back
    return seen


def existing_layouts(pts, s):
    """Station lists (metres) of every dev layout we must not repeat."""
    n = len(pts)
    out = []

    def from_index(first, stride, count):
        out.append(sorted(float(s[(first + k * stride) % n]) for k in range(count)))

    blind = json.loads((ROOT / 'runs/berlin-blind-300/layouts.json').read_text())
    for first, stride, offsets in blind.values():
        from_index(first, stride, len(offsets))
    brake = {'original': (94, 30, 3), 'mirror': (94, 30, 3), 'tight': (124, 18, 3),
             'same-side': (304, 32, 3), 'mid-bends': (197, 40, 3), 'distributed': (102, 142, 3)}
    for first, stride, count in brake.values():   # copied from runs/berlin-brake-merge/ros.py
        from_index(first, stride, count)
    scout = json.loads((ROOT / 'runs/berlin-scout-speed-plan/layouts.json').read_text())
    for v in scout.values():
        out.append(sorted(v['stations']))
    from_index(94, 30, 3)                           # README example, seed 2026
    return out


def circ(a, b, length):
    d = abs(a - b) % length
    return min(d, length - d)


def repeats(stations, existing, length):
    for ex in existing:
        if len(ex) != len(stations):
            continue
        if all(min(circ(a, b, length) for b in ex) < MIN_EXISTING_GAP for a in stations):
            return True
    return False


def offsets_for(name, rng, count=5):
    if name == 'D':
        sign = 1 if rng.random() < .5 else -1
        return [round(float(sign * (-1) ** k * rng.uniform(.14, .20)), 4) for k in range(count)]
    if name == 'E':
        return [round(float(rng.uniform(-.08, .08)), 4) for _ in range(count)]
    return [round(float(rng.uniform(-.12, .12)), 4) for _ in range(count)]


def check(grid, pts, s, length, idx, offs):
    """Passability: every obstacle leaves PASS_WIDTH + EXTRA_MARGIN on some side, and close
    pairs (< SWITCH_S along s) share a free side."""
    need = PASS_WIDTH + EXTRA_MARGIN
    info, free_sides = [], []
    for i, d in zip(idx, offs):
        left, right = side_gaps(grid, pts, i, d)
        sides = {k for k, g in (('left', left), ('right', right)) if g >= need}
        info.append(dict(left_gap_m=round(left, 3), right_gap_m=round(right, 3),
                         free_sides=sorted(sides)))
        free_sides.append(sides)
    ok = all(free_sides)
    order = sorted(range(len(idx)), key=lambda k: s[idx[k]])
    for a, b in zip(order, order[1:]):
        if circ(s[idx[a]], s[idx[b]], length) < SWITCH_S and not (free_sides[a] & free_sides[b]):
            ok = False
    return ok, info


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--check-only', action='store_true', help='re-check an existing layout file')
    args = ap.parse_args()

    pts, s, length, vx, ax = load_track()
    n = len(pts)
    grid = Grid()
    kappa = np.array([route_curvature(pts, i) for i in range(n)])
    def allowed(i, start=START_INDEX):
        return circ(s[i], s[start], length) >= MIN_START_GAP

    if args.check_only:
        spec = json.loads(OUT.read_text())
        bad = 0
        for name, lay in spec['layouts'].items():
            for count, cfg in lay['configs'].items():
                idx = [(cfg['obstacle_index'] + k * cfg['obstacle_stride']) % n for k in range(int(count))]
                ok, _ = check(grid, pts, s, length, idx, cfg['obstacle_offsets'])
                ok = ok and all(allowed(i, cfg['start_index']) for i in idx)
                bad += not ok
                print(name, count, 'ok' if ok else 'FAIL')
        raise SystemExit(1 if bad else 0)

    # blind-corner feature: visibility of a centred obstacle, precomputed per waypoint
    vis = np.array([visibility(grid, pts, s, length, i, 0.) for i in range(n)])
    existing = existing_layouts(pts, s)

    def score(name, idx, offs):
        w = np.array([3., 2., 2., 1., 1.])
        k = np.abs(kappa[idx])
        if name == 'A':
            gaps = np.diff(sorted(s[idx].tolist()) + [sorted(s[idx].tolist())[0] + length])
            return -np.std(gaps) - 0.2 * np.std(k)
        if name == 'B':
            return float((w * np.minimum(k, .6)).sum())
        if name == 'C':
            return float((w / (vis[idx] + .5)).sum())
        if name == 'D':
            margins = []
            for i, d in zip(idx, offs):
                left, right = side_gaps(grid, pts, i, d)
                margins.append(right if d > 0 else left)   # the side the car is pushed to
            return float(min(margins)) - 0.3 * float(np.mean(k))
        if name == 'E':
            brake = np.clip(-ax[idx], 0, 10) / 10
            return float((w * (vx[idx] / vx.max() + .5 * brake)).sum())
        raise KeyError(name)

    result = {
        'map': 'berlin', 'waypoints_file': 'maps/berlin/global_waypoints.json',
        'lap_length_m': round(float(length), 3), 'waypoints': n,
        'generator': 'tools/longrun_layouts.py',
        'checks': dict(passable_width_m=PASS_WIDTH + EXTRA_MARGIN, min_start_gap_m=MIN_START_GAP,
                       min_existing_gap_m=MIN_EXISTING_GAP, switch_s_m=SWITCH_S,
                       obstacle_size_m=OBSTACLE_SIZE),
        'note': 'Test scenarios only (el_scenario). The controller never reads this file.',
        'layouts': {},
    }
    for name, spec in LAYOUTS.items():
        rng = np.random.default_rng(spec['seed'])
        start = spec.get('start', START_INDEX)
        offs = offsets_for(name, rng)
        best = None
        for stride in spec['strides']:
            for first in range(n):
                idx = [(first + k * stride) % n for k in range(5)]
                if not all(allowed(i, start) for i in idx):
                    continue
                if len({i for i in idx}) < 5:
                    continue
                sc = score(name, idx, offs)
                if best is not None and sc <= best[0]:
                    continue
                ok, _ = check(grid, pts, s, length, idx, offs)
                if not ok:
                    continue
                if any(repeats(sorted(s[idx[:c]].tolist()), existing, length) for c in (1, 3, 5)):
                    continue
                best = (sc, first, stride)
        if best is None:
            raise SystemExit(f'layout {name}: no valid placement')
        _, first, stride = best
        configs = {}
        for count in (1, 3, 5):
            idx = [(first + k * stride) % n for k in range(count)]
            ok, info = check(grid, pts, s, length, idx, offs[:count])
            assert ok
            obstacles = []
            for i, d, extra in zip(idx, offs[:count], info):
                obstacles.append(dict(index=int(i), s_m=round(float(s[i]), 3), offset_m=d,
                                      region=road_region(kappa[i]),
                                      curvature=round(float(kappa[i]), 3),
                                      raceline_vx_mps=round(float(vx[i]), 2),
                                      raceline_ax_mps2=round(float(ax[i]), 2),
                                      visibility_m=round(float(vis[i]), 2), **extra))
            configs[str(count)] = dict(start_index=int(start), obstacle_index=int(first),
                                       obstacle_stride=int(stride), obstacle_offsets=offs[:count],
                                       obstacles=obstacles)
        result['layouts'][name] = dict(seed=spec['seed'], character=spec['character'],
                                       configs=configs)
        print(name, 'index', first, 'stride', stride, 'stations',
              [o['s_m'] for o in configs['5']['obstacles']],
              [o['region'] for o in configs['5']['obstacles']])
    OUT.write_text(json.dumps(result, indent=2) + '\n')
    print('wrote', OUT)


if __name__ == '__main__':
    main()
