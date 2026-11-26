# Eval prep status

## Task 0: official obstacle behavior (done, waiting on the instructor)

- Result: `race_manager` **re-draws every obstacle at each completed lap** (on `/sim/lap_count`). No parameter turns this off. `seed` (launch default -1, random) only makes the sequence of layouts repeatable.
- Lap 1 uses layout 0. Every later lap uses a new layout. The lap-1 obstacle memory is out of date from lap 2 onward.
- The shipped package is byte-identical to the one in `misys:forza_full`. It was not modified.
- `forza_full` has no `berlin` map under `stack_master/maps`, so the shipped launch can't run with `map_name:=berlin`.
- Details: `docs/eval-obstacle-behavior.md`
- Blocked on: the instructor's answers to its 4 questions (launch as shipped? seed? parameters? Berlin waypoints file?).

## Task 1: long-run matrix (in progress)

Setup (2026-11-25):
- The runs use their own container, `routertln-longrun` (image `routertln:local`). It has network `none`, `ROS_DOMAIN_ID=57`, a frozen copy of the build in volume `routertln-longrun-build`, and `/workspace` = `/home/zoe/build/RouterTLN`. This keeps the manual race_manager session in `routertln` (domain 56) out of the test runs.
- Warning: run `berlin-215308` in `/home/zoe/RouterTLN/runs` had `el_scenario` with 3 obstacles *and* the manual `obstacle_spawner` publishing `/sim/obstacles` at the same time. It ended with `command_footprint` at 7 m. Don't trust results from a session with both publishers running.
- **The first run in a fresh container fails with `sensor_timeout`.** The gym's numba JIT compiles its kernels (`f110_gym/envs/__pycache__/*.nbi`) during the first drive, and that stalls the bridge past the 150 ms sensor limit. Runs pass once the cache is warm. This VM's CPU is `QEMU Virtual CPU 2.5+` (no AVX). **This matters for Task 3:** the install script must warm the gym cache, or a grader's first demo run may stop at the start.
- Baseline on this VM: `new-1-051`, 4.25 m/s, 3 laps → 23.23 / 14.28 / 14.28 s, clean. Compute p99 12.2 ms, max 14.4 ms.
- Config used for every run: the 4.25 m/s first-lap plan (`FINAL_CONFIG` in `runs/longrun/run.py`).

New files:
- `tools/longrun_layouts.py` generates `config/longrun_layouts.json`.
- `runs/longrun/run.py` is the runner. `runs/longrun/summarize.py` writes `summary.csv` and `summary.md`.
- `scripts/smoke.sh` gets an opt-in `EL_SCENARIO_OBSTACLES=0`, which remaps `el_scenario`'s obstacle topic so race_manager is the only publisher. The default is unchanged.

Smoke tests (`runs/longrun-smoketest`), layout A, 3 obstacles, 10 laps:
- F: 10/10 clean. Laps about 16.9 s, so only **9 laps finish within 180 s**.
- R: 10/10 clean. Best lap 16.27 s, and the official scorer agrees.

## Task 3: install_final_demo.sh (written, clean-container test pending)

- `CMakeLists.txt`: new option `-DEL_CONTROLLER_ONLY=ON`. It builds `el_core`, `el_policy`, `el_controller`, `el_scenario` and `el_inspect`, plus the CPU tests (checkpoint, scan localizer, 8 corridor-entry replays). It needs no CUDA language and no `CUDAToolkit`. The default (OFF) builds the same targets as before.
- `config/final_demo.env`: the one driving config. Both `run_demo.sh` and `runs/longrun/run.py` read it, and it matches the values the matrix runs use.
- `install_final_demo.sh`: needs root (it re-runs itself with sudo), skips finished steps on a re-run, and stops on the first error naming the step and the log. It installs apt packages and LibTorch 2.9.1+cpu (`/opt/libtorch-cpu`). It gets the code with a git clone (`ROUTERTLN_REF`, default `main`) or copies the checkout it sits in. It links `/workspace` and `/build`, then builds and tests. **It also warms the gym numba cache**, then installs `~/run_demo.sh`.
- `run_demo.sh [obstacles] [seed] [laps]`: starts the simulator, the RouterTLN controller, the race_manager spawner and scorer, and (with a display) RViz using the shipped `forza_rviz.rviz` plus map_server and robot_state_publisher. This is the same setup as Mode R in Task 1.
- Pending: the test in a fresh `misys:forza_full` container (`docs/install-test.log`). It waits until the Mode F 10-lap batch finishes, because a parallel build would cause deadline misses in the matrix.
- Open: graders copy only the script, so it clones GitHub. Until these files are committed and pushed, the clone path can't be tested end to end, and the local test uses `ROUTERTLN_SRC`. Before submission, pin `ROUTERTLN_REF` to the final commit.

## Task 5: hardcoding audit (done)

- `docs/hardcoding-audit.md`: read all 15 driving-path files in full and re-ran the grep sweep. There are no section-specific constants: no waypoint-index or x/y literals, no station thresholds, no position-keyed tables. Every hit is relative to the car or is a bounds check. Every tuned constant is listed with its physical meaning.
- Things to explain if asked: the per-waypoint arrays (`scout_lateral_`, `race_speed2_`, `previous_`) are learned during the run, and the two map-wide scalars in `router.yaml` apply to the whole map.
- **Risk found:** `validate_sim_band()` rejects the final config (`sim_section_speed_profile`) unless the controller runs with our local steady sim clock (`local_gym_bridge.py` + the `el_scenario` clock). On the stock `base_system` simulator the controller would not start. Ask the instructor whether evaluation runs our launcher (together with the Task 0 questions).

## Task 2: scan noise robustness (set up, runs queued)

- How noise is set: in the shipped Gym, `RaceCar` calls `ScanSimulator2D.scan(pose, rng)` without `std_dev` (`f110_gym/envs/base_classes.py:459`), so the default **σ = 0.01 m** (`laser_models.py:434`) is fixed. No sim.yaml, sim_params.yaml or launch parameter exposes it, so it can't be changed through config alone.
- Injection (repo only, Gym untouched): `scripts/local_gym_bridge.py` takes an opt-in `EL_SCAN_NOISE_SCALE=k`. It adds independent Gaussian noise of 0.01·√(k²−1) m to the published `/scan` after the Gym step, for a total σ of k cm. Physics, collision checks and the judge use the Gym's internal state and are unaffected. With the default k = 1 the code path is a no-op. Also fixed a latent `np` shadowing bug in the same `__init__`: without `EL_SCAN_NOISE_SEED`, the new code would have raised `UnboundLocalError`.
- Runner: `python3 runs/longrun/run.py --mode F --laps 10 --obstacles 3 --seeds 51001 --noise-scale 2` (and `3`). Output goes to `runs/longrun-20261125/F-noise2x/` and `F-noise3x/`, and the baseline is the matching `F/*-obs3-10L-s51001` runs.
- Queued after the Mode F 10-lap batch (CPU contention). Metrics: `remembered_cells` per lap compared with the baseline (phantom cells), stops, lap times.

## Task 4: submission README (drafted, waiting on team input)

- `SUBMISSION_README.md` has the four required headings: `[레이스 스택 사용법]` (copy-paste install + `~/run_demo.sh 1/3/5`, no `source` steps), `[변경 사항들]` (everything the installer adds to `misys:forza_full`, checked against a clean container; Gym, race_manager and forza_ws untouched), `[내부 조사가 요구되는 파일들]`, and `[자체 평가 결과]`. It also has the "Why two networks" section (both CSVs, with the training-order/WSP confound stated) and the ground-truth audit (controller subscribes only `/scan`, `/clock`, `/parameter_events`).
- `install_final_demo.sh` copies it to `/home/misys/README.md` (the base image has none).
- `README.md`: removed the sentence saying the team-leader and co-work roles "need team confirmation".
- **Waiting on:** (1) the member→file/function mapping for `[내부 조사가 요구되는 파일들]`; the table currently shows git authors only, with owners marked "확인 필요". (2) The `final_demo.mp4` run for `[자체 평가 결과]`.
- Risk: git history has 11 commits by 2 identities. All code arrived in a few large "publish" commits by `pineapple4566`, and `KN_Yeon`'s commits touch README/docs only. If other members did code work, the history won't show it.

## Task 6 (optional): fair unified-vs-split comparison (blocked: training data not on this machine)

- The split pair: Race = `race-teachers-expanded-v5.pt` (20 epochs from scratch, 16 train / 6 val episodes of `data/teachers-expanded-v5-20261029/accepted/race-*.elds`, lr 1e-4) + the WSP edit (`runs/race-wsp-berlin/make_edit.py`, gain 12) → `race-berlin-wsp.pt`. Avoid = `avoid-corridor-v2.pt` (16 epochs, lr 1.5e-4, fine-tuned from `avoid-chain-v2.pt` on `data/avoid-corridor-replay-v2/` 46 train / 12 val), no WSP.
- The existing unified model: `unified-race-avoid-v2.pt` (16 epochs from `runs/unified-race-avoid-v2/base.pt`, 62 train / 18 val episodes = both sets). So the training order, the starting checkpoints and the WSP scope all differ.
- **Blocked:** no `.elds` data on this VM (`data/` is gitignored; neither clone, the build volume nor the filesystem has any). The docs place the originals in `el-project/native/...` on another machine. Regenerating isn't equivalent: 843.51 MiB / 276 files from the CUDA MPPI teacher, with SHA256s on record, and GPU collection is not bit-reproducible.
- Protocol once the data is copied to `data/` (same paths as the training JSONs), about 1 min of GPU training each, run after the matrix:
  1. Check the SHA256s against the stored lists.
  2. Train unified with the split pair's recipe: Race episodes from scratch for 20 epochs at 1e-4, then fine-tune on Race+Avoid episodes for 16 epochs at 1.5e-4 (mirroring Avoid's fine-tune). Seed 2026, same validation split, same lowest-val-loss selection.
  3. Apply the same WSP edit (gain 12 on `context[6]`) to the unified steering head.
  4. Rerun the 18 layouts with `el_eval_avoid` and `runs/berlin-brake-merge/paired.json`-style cases from `docs/berlin-policy-comparison.csv`, plus the 5 ROS layouts. Only the checkpoint changes.

## localization_lost fix (approved: diagnostics + options 3 and 1), code ready, build/runs after the Mode F batch

- `include/el/scan_localizer.hpp`: each rejection now records its reason (`last_rejection()`). New option `set_measured_speed_prior()` builds the motion prior from the estimated speed instead of the commanded speed (option 3). New `coast()` advances the pose by the prior for a rejected scan.
- `src/node.cpp`: logs `localizer rejected scan stamp=…: <reason>` (diagnostic, no behavior change). New parameters `localizer_measured_speed_prior` (bool) and `localization_grace_scans` (0–2), **both default off = the behavior evaluated so far**. With grace on, the router and command guard still run on the coasted pose, and the coasted scan is not written to obstacle memory.
- `scripts/smoke.sh`: pass-through `EL_LOCALIZER_MEASURED_SPEED_PRIOR`, `EL_LOCALIZATION_GRACE_SCANS`. `runs/longrun/run.py`: `--variant <label> --set KEY=VALUE`. `runs/longrun/summarize.py`: new columns `max_along_err_m`, `p99_along_err_m`, `loc_rejections`, `loc_coasts`.
- **Incident:** editing `smoke.sh` in place broke the run that was executing at the time (`F/B-obs1-10L-s51002`, bash syntax error before scoring). It was moved to `runs/longrun-20261125/_broken/` with a note and will be rerun. From now on, scripts in use are replaced atomically (temp file + rename).
- Plan: (1) patched controller built into `/opt/build-locfix` inside `routertln-longrun`, leaving `/build` untouched. (2) Diagnostic: rerun `A-obs1-10L-s51002` with the patched binary and defaults, to read the rejection reason. (3) Variant `locfix` = `EL_LOCALIZER_MEASURED_SPEED_PRIOR=true EL_LOCALIZATION_GRACE_SCANS=2` on the full Mode F 10-lap matrix plus 2×/3× noise. Compare with baseline on stops, `max_along_err_m`, lap times and compute.
- Added (approved): `deadline_grace_cycles` (0/1, **default 0 = current behavior**, passed through as `EL_DEADLINE_GRACE_CYCLES`). With 1, one isolated late control cycle doesn't latch STOP. It still publishes the guard-validated command and still has to pass the scan-age (150 ms) and clock checks. A second consecutive late cycle latches `deadline` as before. Every deadline miss is now logged (it wasn't before), and the summary counts `deadline_tolerated`. Trigger: `F/C-obs5-10L-s51001` stopped in lap 4 on one 44.5 ms cycle, 38 ms of it in input packing (CPU time ≈ wall time, which points to VM steal; not proven, since per-second `vmstat` logging only started at 23:31).
- Test variant `locfix` = `EL_LOCALIZER_MEASURED_SPEED_PRIOR=true EL_LOCALIZATION_GRACE_SCANS=2 EL_DEADLINE_GRACE_CYCLES=1`.
- Added (approved): `fallback_budget_ms` (**default 0 = unlimited = current behavior**, `EL_FALLBACK_BUDGET_MS`). It bounds the Avoid fallback search in `SectionRouter::step` (2 speeds × 7 steering candidates, each a full `command_refusal` rollout). Trigger: `F/D-obs3-10L-s51002` lap 9, a 47.5 ms cycle with 39.5 ms in `safety`, in Avoid mode, with 0 % steal on the vmstat log. The CSV reason `deadline` means the search had *succeeded*, only late, so a plain cutoff would just turn it into a refusal stop. When the budget runs out, one guard-checked brake command (`budget_brake`, queued steering at speed 0) is published instead of latching; if that fails too, the stop latches as before. Summary column `budget_brakes`.
- Test variant `locfix` = `EL_LOCALIZER_MEASURED_SPEED_PRIOR=true EL_LOCALIZATION_GRACE_SCANS=2 EL_DEADLINE_GRACE_CYCLES=1 EL_FALLBACK_BUDGET_MS=15`.

## Task 3 test result (2026-11-26)
- Clean-container install: OK (1 min 39 s, ctest 10/10, warm-up 19 kernels), idempotent re-run OK (2 s). Transcript: `docs/install-test.log`. The clone path is untested until the branch `Juhwan_Kim-eval-prep` is pushed. The RViz path is untested (no display).
- First demo after install: no first-run sensor_timeout (warm-up works). **Wall contact in lap 5** from a Race-mode guard refusal plus a stop command that feeds back the estimated steering: `docs/demo-wall-contact-analysis.md`. The summary print in `run_demo.sh` was fixed (the scorer loses its printout on shutdown).

## Mode F 10-lap matrix (done)
- 30/30 runs, 0 collisions, 27/30 without a stop. Stops: `localization_lost` (A-obs1-s51002, lap 2), `deadline` (C-obs5-s51001 lap 4: 38 ms in packing; D-obs3-s51002 lap 9: 39.5 ms in the Avoid fallback search). No drift with lap count (laps 1–5 vs 6–10: mean lap 16.13 / 16.08 s, compute p99 12.5 / 12.7 ms). `runs/longrun-20261125/summary.{csv,md}`.
- 2026-11-26: diagnostic rerun did not reproduce localization_lost (no rejections in 5 laps); it hit a 54 ms Race-mode deadline instead. Deadline stalls are likely VM memory pressure (direct reclaim/compaction), not the fallback search; see docs/localization-lost-analysis.md. Stall probe running. Task 2 noise runs (2x, 3x; baseline binary) started.
- Added (approved): `stop_hold_steering` (`EL_STOP_HOLD_STEERING`) holds the last published steering in every stop (router stop, node fault/deadline stop, `refuse()`), matching the guard's braking model. `race_fallback` (`EL_RACE_FALLBACK`) runs the fallback search plus `budget_brake` in Race mode too. **Both default off.** Trigger: the wall contact in the post-install demo (`docs/demo-wall-contact-analysis.md`).
- `locfix` variant (final): `EL_LOCALIZER_MEASURED_SPEED_PRIOR=true EL_LOCALIZATION_GRACE_SCANS=2 EL_DEADLINE_GRACE_CYCLES=1 EL_FALLBACK_BUDGET_MS=15 EL_STOP_HOLD_STEERING=true EL_RACE_FALLBACK=true`, binary `/opt/build-locfix/el_controller` (rebuilt after the noise runs).

## Overnight queue (started 2026-11-26 00:28, `runs/longrun/overnight.sh`, log `/home/zoe/build/overnight.log`)
1. Wait for the Task 2 noise runs (baseline). 2. Build locfix. 3. Mode F 10-lap locfix (30). 4. Noise 2×/3× locfix (10). 5. Demo crash case (Mode R, seed 2026, 1 obstacle), baseline + locfix. 6. Mode R 10-lap baseline (30). 7. Mode R 10-lap locfix (30). 8. **First zip** (`/home/zoe/build/RouterTLN-eval-prep-<stamp>.zip`). 9. 20-lap A/C/E baseline + locfix (18). 10. **Final zip.** Each step continues on failure and is logged here. Estimated ≈ 8–9 h in total; the first zip after ≈ 6–7 h.
- 2026-11-26 00:43:36 overnight: Task 2 noise runs (baseline) finished
- 2026-11-26 00:48:19 overnight: Mode F 10-lap locfix finished
- 2026-11-26 00:49:50 overnight: noise runs locfix finished
- 2026-11-26 02:20:23 overnight: Mode R 10-lap baseline finished
- 2026-11-26 02:26:26 overnight: Mode R 10-lap locfix finished
- 2026-11-26 03:21:20 overnight: 20-lap A/C/E finished; queue complete
