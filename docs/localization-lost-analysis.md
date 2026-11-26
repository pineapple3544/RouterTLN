# `localization_lost` in Mode F run A-obs1-10L-s51002

Run: `runs/longrun-20261125/F/A-obs1-10L-s51002` (layout A, 1 obstacle, noise seed 51001+1, final 4.25 m/s config). The run stopped at progress 77.6 m (lap 2, track s ≈ 32.3 m), with no contact.

## What the logs show

From the controller pose in `run.csv` against the physics ground truth in `run-judge.csv` (scoring-side analysis only):

1. **Along-track drift.** From s ≈ 28 m to s ≈ 32 m, while the car accelerates from 2.8 to 4.1 m/s out of the corner complex onto the long, almost straight section (κ ≈ −0.2), the estimate runs **ahead** of the true pose. The along-track error grows from +0.02 m to **+0.15 m**, while the lateral error stays under 0.04 m.
2. **One rejected scan.** At stamp 7856.987 one scan was rejected (a 50 ms gap in the 25 ms control steps). The estimate jumped back by about 0.06–0.09 m (along-track error 0.074 → 0.012 m).
3. **Latched stop.** `node.cpp` treats the first rejected frame while locked as `localization_lost` and latches STOP. The following scans track within 0.004–0.012 m, so the localizer had recovered immediately.

## It happens on every run

| Run | max along-track error | where (> 8 cm) |
|---|---|---|
| A-obs1-s51001 (clean 10 laps) | 0.180 m | s 28–32 m |
| A-obs1-s51002 (**stopped**) | 0.142 m | s 30–32 m |
| A-obs3-s51001 | 0.149 m | s 30–32 m |
| A-obs3-s51002 | 0.187 m | s 30–32 m |
| A-obs5-s51001 | 0.132 m | s 28–32 m |
| A-obs5-s51002 | 0.124 m | s 28–30 m |

The drift is at the same place on every lap, at both noise seeds and with 1, 3 or 5 obstacles. Whether it ends in a stop depends on how large the final correction is, so each lap through s ≈ 30 m is another draw.

## Likely mechanism (`include/el/scan_localizer.hpp`)

- **Drift:** the prior is `predict_command_tick(pose_, previous, dt)`, which accelerates toward the *commanded* speed. While accelerating, the command is ahead of the actual speed (target 3.2–4.2 vs actual 2.9–4.1 m/s), so the prior overshoots. On a near-straight section the walls constrain lateral position and heading well, but along-track position only weakly, so `refine()` (window 0.18 m) doesn't remove the overshoot.
- **Rejection:** when the fit finally snaps back by 6–9 cm in one 25 ms frame, the pose-derived forward speed differs from the prior by ≈ 3 m/s and triggers `fabsf(v-next.v)>3.f`. The motion-constrained fallback only accepts backward jumps (`v>=0 … return {}`), so the frame is dropped.
- **Not yet confirmed:** the localizer doesn't report which check rejected the frame. `fit_loss > 0.065` on a noisy frame is the other candidate. Confirming it needs one diagnostic log line (no behavior change) and a rerun.

## Fix options (each changes driving behavior, so each needs approval)

1. **Tolerate a short gap:** allow up to 2 consecutive rejected scans (≤ 75 ms, under the 150 ms sensor limit), coasting on the motion prior and keeping the command guard, before latching `localization_lost`. This addresses the stop, not the drift.
2. **Accept forward snap-backs:** allow the constrained-motion fallback when the implied speed is lower but still forward (0 ≤ v < prior), bounded by the same 0.18 m / one-cell checks.
3. **Reduce the drift:** build the prior from the estimated speed (or blend the commanded and estimated speed) instead of the raw commanded acceleration.

Each would be tested on the same matrix (layouts A–E × 1/3/5, both noise seeds), plus the 2×/3× noise runs from Task 2, since higher noise makes this failure more likely.

## Diagnostic rerun (patched binary, all new options off), 2026-11-26

`runs/longrun-20261125/F-diag/A-obs1-10L-s51002`: **no scan was rejected in 5 laps.** The failure is not deterministic for a given noise seed (control timing differs run to run). The run stopped in lap 6 on a **deadline** instead: 54.0 ms, with 52.0 ms in `safety` **in Race mode**. Race mode has no fallback search (normally 0.1–0.4 ms there). CPU time ≈ wall time, steal 0 % in the per-second vmstat. The rejection logging is in place for the `locfix` matrix.

## Correction: the deadline stalls are probably not our algorithm

All three deadline stalls (C-obs5: 38 ms in `packing`; D-obs3: 39.5 ms in `safety`, in Avoid; diag: 52 ms in `safety`, in Race) put the time in an arbitrary stage, charged as thread CPU, with 0 % steal. Earlier I attributed D-obs3 to the Avoid fallback search, but the identical Race-mode stall (no search there) makes that doubtful. The VM has been short on memory for most of the campaign: free < 400 MB for ≈ 46 min of the vmstat log, 1.4 GB swapped out, `allocstall` 63, `compact_stall` 14 since boot. My containers use ≈ 850 MB; the host desktop and the page cache take the rest. Direct reclaim and compaction run in the allocating thread and count as its CPU time, which fits. Not proven yet: an independent stall probe (`stall_probe.py`, 1 ms busy chunks, logs > 10 ms) now runs alongside the next runs. The deadline grace cycle helps whatever the cause. The 15 ms fallback budget remains a sensible bound, but it may not be what addresses these stalls.
