# Wall contact in the first post-install demo (Mode R, 1 obstacle, seed 2026)

Run: `~/run_demo.sh 1` in the install-test container (`/workspace/runs/demo-20261126-001425-obs1-seed2026`; copy in the session scratchpad). race_manager: 4 clean laps (best 15.66 s), then **1 wall contact** in lap 5 at x 6.07, y −4.35.

## Timeline (controller CSV vs physics judge)

- Track s ≈ 41.3–42.0 m, the right-hand side is narrow (`d_right` 0.53 m). Lap 5's race_manager obstacle was at s ≈ 18.5 m and played no part.
- Until step 3422 the car is in Race mode, 4.0–4.2 m/s, on the raceline (lateral −0.01…−0.06 m), steering −0.02…−0.10 rad, localizer error 1–6 cm. Nothing abnormal.
- **Step 3423:** the command guard refuses with `dynamic_braking_footprint`. Braking now with steering *held* at ≈ −0.09 rad would curve the ≈ 2 m stopping arc into the narrow right wall. Race mode has no fallback search (that exists only in Avoid with lap planning), so the stop latches.
- **The latched STOP steers into the wall.** `SectionRouter::stop_with()` returns `{car.steer, 0}`, and `car.steer` is the localizer's estimate `atan(0.3302·yaw_rate / v)`. While braking, v falls faster than the yaw rate, so the published steering climbs −0.09 → −0.17 → −0.26 → −0.33 → −0.38 rad. The car drifts right (lateral −0.01 → −0.14 m) and touches the wall at 1.6 m/s, 0.31 s after the refusal.

## Two defects

1. **The stop command doesn't match the guard's stop model.** The guard checks braking with the steering *held* at the committed angle. The executed stop instead feeds back the estimated steering, which grows as v → 0 and approaches the ±0.4 clamp.
2. **No Race-mode fallback.** One refused Race command latches STOP immediately, even when a slower or less-steered command (or a brake with a different steering angle) would keep the stopping arc clear.

## Fix options (change driving behavior, need approval)

- A. **Hold the last published steering** during a latched stop, i.e. `last_command_.steer` (= `committed.steer`) instead of `car.steer`. That's exactly what the guard's braking rollout assumes.
- B. **Run the same fallback search in Race mode** (2 speeds × 7 steering candidates plus `budget_brake`, under the same `fallback_budget_ms`) before latching.

None of the 30 Mode F 10-lap runs hit this, but it's a property of the Race path, not of Mode R.
