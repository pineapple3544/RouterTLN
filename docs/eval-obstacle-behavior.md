# How the official obstacles behave (Task 0)

Checked 2026-11-25 by reading the shipped `race_manager` source (read only, nothing changed).

- Source: `/home/misys/forza_ws/race_stack/utilities/nodes/race_manager/`
- Installed config: `$(ros2 pkg prefix --share race_manager)/config/race_manager_params.yaml`
  = `/home/misys/forza_ws/race_stack/install/race_manager/share/race_manager/config/race_manager_params.yaml`
- The package in `routertln:local` is byte-identical to the one in `misys:forza_full`.
  Both trees have tar md5 `b5a708d99c698513e5742beed74c2181`, and the per-file md5s match.
  The installed Python package is also identical to its source.

## Answer

**The obstacles change every lap.** The shipped code has **no parameter** that keeps them fixed for a whole run.

| Scope | What changes | Controlled by |
|---|---|---|
| Per lap | All obstacle positions and rotations, plus which recipe is used | Hard-coded. `lap_callback` calls `place()` whenever `/sim/lap_count` changes |
| Per run | The whole sequence of layouts | `seed`. `>= 0` repeats the same sequence, `-1` picks a random seed each run |
| Per config | The candidate pool and the recipe mix | The generation parameters below. A different value changes every seed's layouts |

### How it works (`obstacle_spawner.py`)

1. At startup it builds the candidate pool. `generate_candidates()` is deterministic given the waypoints and the parameters.
2. It seeds `random.Random(seed)` and immediately places layout 0 (`self.lap = 0; self.place()`).
3. It subscribes to `/sim/lap_count` (`Int32`). In `lap_callback`, if `msg.data != self.lap`, it calls `place()` again. That picks a new recipe with `rng.choice(recipes)` and draws a new set of obstacles.
4. It publishes to `/sim/obstacles` (`PoseArray`, latched) and re-sends the same layout every 1 s.

### When the re-draw happens (`scorer.py`)

- **First line crossing:** the scorer starts timing and publishes `lap_count = 0`. That equals the spawner's starting value, so **nothing is re-drawn**. Layout 0 covers the drive from the start position to the line, plus the first timed lap.
- **Each completed lap** (the line is crossed after covering at least half the track length): `lap += 1` is published, and **the spawner re-draws at the moment the car crosses the line.**
- A crossing after less than half a lap is ignored, so there is no re-draw.
- After a teleport via `/initialpose`, the lap is voided. The next credited lap still increments `lap_count`, so a re-draw still happens.
- Effect for our controller: **lap 1 is driven against layout 0, lap 2 against layout 1, and so on.** The obstacles that lap 1 "learned" are gone from lap 2 onward.
- Guarantees at the re-draw: nothing is placed within `exclusion_ahead` = 5.0 m in front of the line, or within `exclusion_behind` = 1.0 m behind it.

## Parameters and defaults

Values are from the shipped `race_manager_params.yaml`. Where the code default in `declare_parameter` differs, it is noted.

### `obstacle_spawner`

| Parameter | YAML value | Notes |
|---|---|---|
| `waypoints_file` | (set by launch) | `stack_master/maps/<map_name>/global_waypoints.json` |
| `num_obstacles` | 4 | The launch arg `num_obstacles` (default 4) overrides it |
| `seed` | (set by launch) | The launch arg default is `-1`, which is random per run. The README says `>= 0` is "for grading" |
| `obstacle_size` | 0.30 | Cube side, in metres |
| `car_width` | 0.31 | |
| `wall_margin` | 0.15 | |
| `squeeze_clearance` | 0.05 | Passable width = 0.31 + 2×0.05 = **0.41 m** |
| `exclusion_ahead` | 5.0 | No obstacle within 5 m after the line |
| `exclusion_behind` | 1.0 | |
| `min_obstacle_gap` | 0.50 | Minimum surface-to-surface distance between obstacles; joint passability is also checked |
| `candidate_spacing` | 0.5 | Spacing along the track (s), in metres |
| `candidate_d_step` | 0.10 | Lateral spacing, measured from the raceline |
| `candidate_thetas` | [0, 22.5°, 45°, 67.5°] | Rotations |
| `min_severity` | 0.0 | |
| `chicane_span` | 2.5 | Maximum s distance between the two halves of a chicane pair |
| `recipes` | 4 entries, one picked per lap | YAML: `spread:1.0 chicane:1:0.10 forced:1:0.20` / `spread:1.0 forced:2:0.20` / `spread:0.5 forced:1:0.30 either:1:0.20` / `chicane:2:0.10`. The code defaults have no `spread` and a different order, but the YAML overrides them |

`num_obstacles` of 1 or 3 is smaller than some recipes need. For example, `chicane:2` needs 4. When a slot doesn't fit, it is skipped and a "recipe relaxed" warning is logged; the spawner keeps running. So 1 and 3 obstacles work, but the mix of obstacle kinds differs from what the recipe asks for.

### `scorer`

| Parameter | Default | Notes |
|---|---|---|
| `teleport_m` | 1.5 | A larger jump voids the lap and counts as a teleport |
| `odom_topic` | `/car_state/odom_GT` | Ground truth is used only for lap counting and scoring |
| `waypoints_file`, `map_name`, `seed`, `log_dir` | from launch | `log_dir` defaults to `race_result` |

### Launch (`race_manager_launch.xml`)

The launch args are `map_name` (default `hyhall12`), `seed` (-1), `num_obstacles` (4), `log_dir` (`race_result`) and `plot` (true). It always starts **both** nodes together, so lap re-draws are always on.

## What this means for RouterTLN

- If evaluation uses `race_manager` as shipped, the obstacles move every lap. The core assumption, that what is learned on lap 1 still holds later, **does not hold for obstacles**. Only the wall and track geometry stays the same.
- This is why **Mode R in Task 1 is the realistic test**, and Mode F (`el_scenario`, fixed) is the optimistic one.
- The only way to get fixed obstacles is a different setup, such as running the spawner without the scorer or using a different publisher. The shipped launch file can't do it, so we should assume evaluation won't.

## Open question for the instructor (must confirm)

1. On evaluation day, is `race_manager_launch.xml` used as shipped, with obstacles re-drawn every lap?
2. Is a fixed `seed >= 0` used? Is it the same seed for the 1, 3 and 5 obstacle runs?
3. Are `num_obstacles` 1/3/5 passed only through the launch arg, with the other parameters at the shipped values (including `recipes` and `exclusion_ahead: 5.0`)?
4. Which `global_waypoints.json` is used for Berlin? `misys:forza_full` has **no `berlin` folder** under `stack_master/maps/` (only GLC_smile_small, glc_ot_ez, hall, hangar_1905_v0, hyhall12, small_hall and teras). So `ros2 launch race_manager race_manager_launch.xml map_name:=berlin` would fail. Our README instead runs the nodes directly with `waypoints_file:=/workspace/maps/berlin/global_waypoints.json`. A different waypoints file gives a different candidate pool, and so different layouts for the same seed.
