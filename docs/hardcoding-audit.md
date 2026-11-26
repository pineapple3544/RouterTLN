# Hardcoding audit: driving path (Task 5)

Audited 2026-11-25 at commit `23fa886` plus the uncommitted eval-prep changes, which don't touch the driving path.

**Scope:** everything `el_controller` executes: `src/node.cpp`, `src/core.cpp`, `src/policy.cpp`, `include/el/{section_router,avoid_corridor,lap_plan,race_speed,scan_localizer,dynamics,core,map_paths,network,control_timing,policy,scenarios}.hpp`, `maps/berlin/router.yaml`. Every file was read in full. The training/teacher code (`teacher.cu`, `mppi.cu`, `collect.cpp`, `eval_*.cpp`) and the test scenarios (`scenario.cpp`, `config/longrun_layouts.json`) are not part of driving.

## Result

**No section-specific constants.** The driving path has no waypoint-index literals, no x/y coordinate literals, no station (`s`) thresholds, no lookahead or speed tables keyed by map position, and no Berlin-specific branches. Every tuned constant is one of:

1. a vehicle or simulator parameter, copied from the shipped `sim_params.yaml` / `sim.yaml`;
2. a speed-dependent physical envelope (stopping distance, lateral or longitudinal acceleration, latency);
3. a distance or offset *relative to the car* or to the planning window that moves with the car;
4. a planner grid resolution or cost weight;
5. a timing or safety threshold.

The same constants apply at every point on the lap, and on any map that supplies `global_waypoints.json` and a map image.

There are three things graders may still ask about. They are explained below:
- **Per-waypoint arrays learned while driving.** The lap plan and the corridor's previous solution hold one value per waypoint, filled in during *this* run. They are not constants.
- **Two map-wide scalars in `maps/berlin/router.yaml`.** They apply to the whole map, not to any section.
- **The default map path** `/workspace/maps/berlin`. It's a path, not a parameter.

## Method

The mechanical sweep below and a full read of each file. The commands can be re-run from the repo root:

```bash
F="src/node.cpp src/core.cpp src/policy.cpp include/el/core.hpp include/el/section_router.hpp \
   include/el/avoid_corridor.hpp include/el/lap_plan.hpp include/el/scan_localizer.hpp \
   include/el/dynamics.hpp include/el/race_speed.hpp include/el/map_paths.hpp include/el/network.hpp \
   include/el/control_timing.hpp include/el/policy.hpp include/el/scenarios.hpp"
grep -nE "points\[[0-9]+\]|wp\[[0-9]+\]|\[ *[0-9]{2,} *\]" $F                       # literal waypoint index
grep -nE "\b(s|station|position_?|index|route_index|n|progress_?|here\.s|ahead)\s*(<|>|<=|>=|==)\s*-?[0-9]" $F
grep -nE "\b(x|y|car\.x|car\.y|pose\.x|pose\.y)\s*(<|>|<=|>=|==)\s*-?[0-9]" $F       # coordinate literals
grep -nEi "berlin|hyhall|section_[0-9]|corner_[0-9]|sector" $F                     # map names
```

| Hit | What it is | Section-specific? |
|---|---|---|
| `avoid_corridor.hpp:179` `prior[15]` | 15th sample of the curved-entry window, about 1.5 m ahead of the car | No: relative to the car |
| `section_router.hpp:236` `ahead <= 1.5f` | merge check over the first 1.5 m ahead of the car | No: relative |
| `section_router.hpp:295` `ahead < 8.f` | narrow-bend scan 8 m ahead (disabled in the final config) | No: relative |
| `core.cpp:79` `x<0 \|\| y<0 …` | grid-cell bounds check | No |
| `map_paths.hpp:8` `"/workspace/maps/berlin"` | default value of the `map_dir` parameter | No: a path |

## Tuned constants and their physical meaning

The constants below are active in the final config (`config/final_demo.env`). Rows marked *(off)* belong to options the final config disables. v is the car's estimated speed (m/s), and κ is path curvature (1/m).

### Vehicle and simulator model (copied, not tuned)

| Constant | Where | Meaning / source |
|---|---|---|
| wheelbase 0.3302 m (lf 0.15875 + lr 0.17145) | `dynamics.hpp`, `core.hpp`, pursuit terms | `sim_params.yaml` |
| body half-length 0.29 m, half-width 0.1016 m | `footprint()` | car length 0.58 / width 0.2032 from `sim_params.yaml` |
| LiDAR 0.27 m ahead of base | `Memory::update`, `observe`, localizer, `gym_front_blocked` | `sim.yaml` `scan_distance_to_base_link` |
| steering ±0.40 rad command clamp, 0.4189 physical limit, 3.2 rad/s slew | everywhere | `s_max`, `sv_max` |
| μ 1.0489, C_Sf 4.718, C_Sr 5.4562, m 3.47, I 0.04712, h 0.074; a_max 9.51, v_switch 7.319 | `Vehicle`, `derivative()` | single-track model, `sim_params.yaml` (see `THIRD_PARTY_DYNAMICS.txt`) |
| drive gains 4.755 / 19.02 / 0.951 / 3.804 | `drive_acceleration()` | Gym speed controller: 10·a_max/v_max, 40·a_max/v_max, … with a_max 9.51 and v_max 20 |
| kinematic predictor: τ = 0.15 s, accel clamp −6…+4 m/s² | `advance()`, `predict_command_tick()` | measured drive response, used only in the safety prediction |

### Section routing (Race ↔ Avoid)

| Constant | Where | Meaning | Why not section-specific |
|---|---|---|---|
| entry distance **2 + 0.45 v + v²/10** m | `route_observation` | switch to Avoid when the first occupied cell on the raceline ahead is within this. 2 m base reaction, 0.45 s of latency travel, v²/(2·5 m/s²) braking | depends only on speed. Example: 4.25 m at 3 m/s, 7.12 m at 5.25 m/s |
| merge distance entry + 0.7 + \|lateral\| m | same | how far past an obstacle the next one still counts as the same section | relative to the car |
| scan horizon min(0.48·L, entry + merge + 5) m | same | look ahead at most half a lap | fraction of the lap length L |
| merge-path check over the first 1.5 m (smoothstep) | same | can the car blend from its lateral offset back onto the line | relative |
| obstacle-layer margin 0.19 m | same | footprint inflation for obstacle cells only | car geometry |
| **return**: past section end + 0.35 m, \|lateral\| < **0.30 m**, \|heading error\| < **0.32 rad**, held **0.20 s** | same | hand back to Race only when re-aligned with the raceline and stable for 8 control periods | thresholds on the car's own tracking error |
| side sampled at first + 0.35 m | `side_at` | which way the corridor passes the obstacle | relative |

### First-lap plan (`lap_plan.hpp`)

| Constant | Meaning |
|---|---|
| scout cap 2.2 m/s (`scout_speed_cap`) | speed limit on lap 1 while the plan is recorded |
| plan lead 0.35 m + 0.25 s·v, approach 0.75 × recorded section speed | when to start a remembered Avoid section |
| confirmation window −1 m … section length + 1 m | a remembered section only switches policy if the *current* scan sees an obstacle inside it |
| section speed profile: cap = √(v_min² + 3.6·d), d = distance − 0.65 m − 0.25 s·v | braking to the Avoid entry speed at 1.8 m/s², with a 0.65 m nose reserve |
| Race speed plan: lateral accel 5.2 / (1 + min(0.6, \|scout deviation\|)) m/s², default deviation 0.30 m; forward/backward pass at +3.0 / −2.6 m/s²; latency travel 0.22 s·v | curvature comes from the waypoint yaw, deviation from how far the car itself left the line on lap 1. Neither is a stored per-section value |
| section entry speed clamped to [0.5, scout cap] | |

### Speed envelope and safety guard (`section_router.hpp`, `race_speed.hpp`, `avoid_corridor.hpp`)

| Constant | Meaning |
|---|---|
| final speed cap 4.25 m/s; Avoid band 2.7–3.0 m/s | the one global config (`final_demo.env`) |
| Race policy scale ×2.15 (router.yaml), nominal policy speed 3 m/s | the Race network outputs a 0–3 m/s scale; rescaled to the cap |
| steering cap √(8·0.3302 / tan δ), blended down to 3.3 m/s when stability = 1 − 2.5·(heading error + steering change) < 1 | v²·tan δ / L ≤ 8 m/s² lateral acceleration; slower while the car is still settling |
| sight limit: decel 6 m/s², latency 0.20 s, nose reserve 0.35 m, cone ±0.12 rad | stop within the free range straight ahead in the current scan |
| exit acceleration ≤ 4 m/s² per step | limits throttle steps after a bend |
| brake preview 0.18 s: slow to max(0.8, v − 0.8); second check at 0.24 s: max(1, v − 1) | roll the command forward. If it would hit, slow down early |
| Avoid corridor speed √(2.2/κ + 6·d), d = distance − 0.20 s·v | 2.2 m/s² lateral, 3 m/s² braking along the planned corridor |
| command guard: footprint over the command interval, then a full stop (v < 0.045 m/s within 300 ticks = 3 s) with kinematic **and** dynamic models | refusal reasons `command_footprint`, `braking_footprint`, `dynamic_*`, `braking_horizon` |
| safe alternative: speed max(0.6, floor, 0.65·min(cmd, v)), steering candidates {cmd, current, 0, ±0.2, ±0.4} | fallback before latching STOP |
| hold-clear-turn (scout only): \|queued steer\| ≥ 0.29, unwinding > 0.018, clearance < 0.12 m | keep the turn if unwinding would cut the stopping clearance |
| *(off)* curve cap 4.5 / 7.0 m/s² lateral, 1.8 / 3.0 m/s² braking; narrow-bend 0.60 m probe, 3.9 m/s, κ > 0.22, 2.4 m/s²; straight boost 3.5 m/s, \|steer\| < 0.09 | disabled by `RACE_LAP_SPEED=true`, `RACE_EXIT_SPEED=false`, `STRAIGHT_BOOST=false` |

### Corridor planner (`avoid_corridor.hpp`)

| Constant | Meaning |
|---|---|
| 32 rows × every 2nd waypoint (≈ 0.2 m each, ≈ 6.4 m ahead); 25 columns × 0.10 m (±1.2 m), refined to 0.05 / 0.025 m | lattice in the raceline frame that moves with the car |
| footprint margin 0.065 m, reserve ramp to 0.16 m from 0.65–1.3 m ahead | clearance from observed and remembered occupancy |
| cost weights: offset² 0.32, narrow-room 12 (room < 0.35 m), gate 48, lap-1 line 3, previous plan 3, near-wall 1.6 / 0.32 at 0.16 / 0.23 m, slope 0.15 / 1.6 | prefer a centred, wide, consistent path |
| gate preview: 42 waypoints, 49 × 0.05 m columns | pick the opening before committing |
| steering preview: candidates 0.1·k (k = −4…4), 0.5 s horizon (up to 1.7 m on sharp turns), bend > 0.60 rad | choose steering by rolling out the vehicle model along the corridor |
| guide lookahead scale × (0.65 + 0.12 v) m (+0.35 / 0.65 × (v − 1.8) above 1.8 m/s), scale 0.50 from router.yaml | point fed to the Avoid network |

### Localization (`scan_localizer.hpp`)

| Constant | Meaning |
|---|---|
| global initialisation: candidate poses at every 4th waypoint, lateral offsets {−0.8, −0.4, 0, 0.4, 0.8} m, 24 headings; accept if fit < 0.055 m and unambiguous (no second fit within 0.006 m that is > 0.7 m or > 0.4 rad away) | searches the whole map. **No start pose, `/initialpose` or ground truth is used** |
| tracking: window 0.18 m, max residual 0.065 m, step clamps 0.12 m / 0.10 rad, yaw-rate limit 14 rad/s, speed consistency 3 m/s, filters 0.82 / 0.18 (speed) and 0.40 / 0.60 (yaw rate) | scan-to-static-map Gauss–Newton |

### Obstacle memory (`core.cpp`)

| Constant | Meaning |
|---|---|
| wall association tolerance **0.03 m** | a return within 3σ of the 1 cm simulator noise of the first mapped wall counts as that wall. **Task 2 checks this under higher noise** |
| log-odds −0.7 free / +1.0 hit, clamped to [−3, 4] | occupancy update |
| beams 181, range clamp 18 m, ≥ 80 % valid beams required | scan preprocessing |

### Timing

| Constant | Meaning |
|---|---|
| control period 25 ms, compute deadline 25 ms (`deadline`) | |
| scan age ≤ 150 ms (`sensor_timeout`, `scan_timeout`) | |
| watchdog: zero command if nothing was published for 180 ms | |

## Per-waypoint state learned while driving

These hold one value per waypoint index. They start **empty** and are filled during the current run, so they are not constants:

| State | Filled by | Reset |
|---|---|---|
| `LapPlan::scout_lateral_[i]` | lateral offset the car itself drove on lap 1 (scout) | `/el/reset_memory`, new run |
| `LapPlan::sections_` | (entry, obstacle, exit) stations measured from **this run's start**, recorded on lap 1 | same |
| `LapPlan::race_speed2_[i]` | computed once from waypoint curvature + `scout_lateral_` + `sections_` | recomputed if the cap changes |
| `AvoidCorridor::previous_[i]` | the planner's last solution (generation-stamped) | each plan |
| `Memory` occupancy grid | the scans | `/el/reset_memory` |

The waypoint list (`global_waypoints.json`) supplies geometry only: x, y, yaw and s, used for the reference line, curvature and the localizer's candidate poses. The raceline speed profile (`vx_mps`) is loaded but **not used** by the controller. Only `Track::problem`/`Track::pursuit` read it, and only the training/teacher code calls those. `maps/berlin/speed_scaling.yaml` belongs to Forza's `speed_scaler`; our code doesn't read it.

## Map-wide settings (`maps/berlin/router.yaml`)

| Key | Value | Meaning |
|---|---|---|
| `race_checkpoint` | `/workspace/models/race-berlin-wsp.pt` | which Race network to load |
| `race_speed_scale` | 2.15 | multiplies the Race network's speed output on the whole map |
| `avoid_lookahead_scale` | 0.50 | scales the Avoid guide lookahead on the whole map |

These apply on every metre of the map. They are the kind of per-map setting a new track would also need. They are not section constants.

## Test-only placement (allowed, not in the driving path)

`src/scenario.cpp` (`obstacle_index`, `obstacle_stride`, `start_index`) and `config/longrun_layouts.json` place obstacles by waypoint index for **test scenarios**. The rules allow this. The controller doesn't read them, and `el_scenario` publishes only to the simulator (`/sim/obstacles`, `/initialpose`) plus the clock.

## Related risk found during the audit (not hardcoding)

`RouterSettings::validate_sim_band()` (`map_paths.hpp:35`) refuses to start with `sim_section_speed_profile=true` (part of the final config) unless the controller runs with `local_steady_sim_clock:=true`. That clock is published by our own simulator launcher: `scripts/local_gym_bridge.py` (a thin wrapper around the shipped `GymBridge`) plus `el_scenario`'s steady `/clock`. **If graders drive our controller against the stock `base_system` simulator, the final config will not start** and exits with "adaptive speed requires a local simulator and a nonempty speed band". `run_demo.sh` uses our launcher, so the demo is fine. Confirm with the instructor that the evaluation runs our launcher, or decide on a fallback config for the stock bridge.
