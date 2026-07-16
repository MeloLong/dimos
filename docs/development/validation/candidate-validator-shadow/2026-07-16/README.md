# Candidate Validator Shadow Batch Report

Date: 2026-07-16  
Branch: `wd/m20-mujoco-simulation`  
Stage 1 commit: `9b6bd880`

## Method

The batch tool published a zero teleop command before every goal so the
planner continued to produce raw and smoothed paths while `MovementManager`
suppressed navigation velocity. The robot remained at approximately
`(-1.006, 1.000)` throughout every run. Each successful goal had exactly one
matching validator shadow record.

The 49-goal grid was repeated three times in two conditions:

- **Dynamic:** the deterministic moving-person module remained active.
- **Static clean:** the moving-person module was stopped after startup and the
  map was allowed to settle before the three sweeps.

The first static attempt is excluded. Its original 50 ms teleop-to-goal delay
allowed one cancellation callback to overlap a new goal. The tool now waits
200 ms; the clean rerun produced no new assertion and zero odometry drift.

## Results

| Metric | Dynamic 3x | Static clean 3x |
|---|---:|---:|
| Goals issued | 147 | 147 |
| Successful plans | 85 | 102 |
| Maximum odometry drift | 0.000 m | 0.000 m |
| Legacy alpha 1.0 selections | 81 | 99 |
| Legacy alpha 0.5 selections | 4 | 3 |
| Hard-invalid shadow candidates | 3 | 18 |
| Selected paths with lower minimum clearance | 24 | 21 |
| Selected paths with lower P5 clearance | 48 | 45 |
| Selected paths with increased unknown length | 14 | 15 |
| Selected paths worse in minimum clearance and unknown length | 3 | 9 |

Selected-candidate changes relative to raw A*:

| Metric | Dynamic median | Dynamic worst | Static median | Static worst |
|---|---:|---:|---:|---:|
| Minimum-clearance loss | 0.000 m | 0.039 m | 0.000 m | 0.017 m |
| P5-clearance loss | 0.006 m | 0.085 m | 0.000 m | 0.092 m |
| Unknown-length increase | 0.000 m | 0.062 m | -0.026 m | 0.033 m |
| Path-length change | -0.076 m | 0.000 m | -0.079 m | 0.000 m |
| Cumulative-turn change | -3.267 rad | 0.000 rad | -4.015 rad | 0.000 rad |

All 34 goals planned in every static round produced exactly identical metric
deltas. Dynamic runs showed expected map-dependent spread: up to 0.024 m for
minimum-clearance loss, 0.091 m for P5 loss, and 0.051 m for unknown-length
increase.

`no_path_or_timeout` means the current map could not produce a raw A* path to
that grid goal within the test timeout. It is not counted as validator failure.

## Findings

1. Shadow mode is decision-neutral and produces repeatable measurements when
   robot pose and obstacle state are fixed.
2. The old `raw_mean_cost + 2.0` gate accepts candidates that lose physical
   clearance or increase unknown-space exposure. Mean map cost is therefore
   not a sufficient safety contract.
3. Strictly forbidding unknown-length increase is measurable, not numerical
   noise: static repeats were identical and deterministic increases reached
   0.033 m.
4. P5 clearance is useful diagnostics but is too sensitive to dynamic-map
   changes to be the first hard gate. Minimum clearance should remain the hard
   clearance quantity.
5. Collision validation works per candidate. The 21 lethal candidates were
   never selected by the legacy decision; shadow evaluation exposed them only
   because it intentionally evaluates every alpha.

## Stage 2 Recommendation

Use these provisional settings for the next shadow-policy replay before
changing live decisions:

- collision and out-of-bounds: unconditional rejection;
- `max_clearance_loss_m`: start at **0.025 m**, one half of the 0.05 m costmap
  cell, then verify narrow passages and real localization/tracking error;
- unknown-length increase: **0.0 m** for normal navigation, with only a tiny
  floating-point epsilon rather than a map-cell allowance;
- P5 clearance and `raw_mean_cost + 2.0`: diagnostics only;
- select the largest passing alpha, otherwise return raw-resampled A*.

A 0.02 m clearance sensitivity check with strict unknown exposure would have
selected alpha 1.0 for 67/85 dynamic and 87/102 static plans, while falling
back to raw for 11 dynamic and 15 static plans. The proposed 0.025 m value must
be replayed against these saved records before activation.

## Artifacts

- `dynamic-3x-report.json`: all dynamic cases and shadow records.
- `static-clean-3x-report.json`: all clean static cases and shadow records.
- `scripts/m20_candidate_validator_sweep.py`: repeatable batch runner.
