# M20 True Simple-Nav Upgrade Log

- Status: Active engineering record
- Baseline: `wd/m20-mujoco-simulation`, commit `5ecbb8e1`
- Scope: MuJoCo `m20-true-simple-nav-sim` planning path quality and stability
- Updated: 2026-07-16

## Purpose

This document records the observed simple-nav behavior, the candidate fixes,
and the changes actually implemented. It separates verified facts from design
options so later tuning and real-robot work can start from a known state.

## Current Planning Chain

```text
point cloud
  -> CostMapper global_costmap
  -> NavigationMap gradient / Voronoi costmap
  -> safe-goal selection
  -> weighted A* global path
  -> constrained local smoothing (M20 MuJoCo profile; legacy fallback otherwise)
  -> uniform resampling (0.1 m)
  -> LocalPlanner tracking controller
  -> nav_cmd_vel
```

The global planner uses a Voronoi-derived costmap for longer goals and a
regular gradient costmap nearer the goal. Low map cost generally means more
clearance from obstacles, but it is not a direct measure of geometric path
length.

## Agreed Single-Plan Quality Program

The current scope deliberately holds the costmap, start, and goal fixed. It
first makes one generated route geometrically suitable for tracking; temporal
consistency between later replans is a separate phase.

### Step 1: Replace The Legacy Smoothing Layer (Current Priority)

Keep weighted A* as the route/topology provider. Replace its unconstrained X/Y
moving average with a local constrained smoother:

```text
raw A* grid path
  -> reference/neighbor iterative smoothing
  -> maximum metric displacement from each raw point
  -> continuous lethal-cell and effective-cost checks
  -> uniform resampling and orientations
  -> LocalPlanner
```

The smoother must keep start/goal fixed, never connect arbitrary distant path
points, and preserve the route corridor with a hard per-point displacement
limit. Unknown cells retain the same effective penalty used by A* rather than
silently becoming free or a new hard obstacle. A failed final validation falls
back to the raw A* geometry with uniform resampling.

The first configuration surface is:

- `constrained_path_smoothing_enabled`
- `path_smoothing_iterations`
- `path_smoothing_data_weight`
- `path_smoothing_smoothness_weight`
- `path_smoothing_max_deviation_m`
- `path_smoothing_collision_sample_spacing_m`
- `path_smoothing_max_cost_increase`
- `path_resample_spacing_m`

Legacy smoothing remains the compatibility default for other
`ReplanningAStarPlanner` users. The M20 MuJoCo profile explicitly enables the
new mode so raw-versus-smoothed Rerun comparison remains available.

### Step 2: Add Heading-Change Cost To A* (Only If Still Needed)

After Step 1 is validated, inspect the orange raw A* path. If it still contains
large alternating direction changes that cannot be smoothed inside the bounded
corridor, extend the A* state from `(x, y)` to `(x, y, incoming_direction)` and
add a small normalized turn cost.

This is intentionally second because it changes search behavior, native/Python
parity, and potentially CPU cost. It is unnecessary when the constrained
smoother can absorb normal grid quantization without violating its deviation
or safety limits.

### Step 1 Implementation And Offline Validation

The first implementation is opt-in at the shared planner level and enabled by
`mujoco_sim.yaml`. It applies only local reference/neighbor corrections; every
raw A* point remains inside a `0.10 m` displacement bound, start and goal stay
fixed, and no distant path points are connected. Each candidate's adjacent
segments are sampled against the same inflated costmap. Lethal or out-of-map
moves are rejected, and an allowed move may increase effective local mean cost
by at most `2.0` cost units. Unknown retains A*'s effective cost of `80`.

The selected MuJoCo profile is 40 iterations, data weight `0.02`, smoothness
weight `0.45`, `0.05 m` collision sampling, and `0.10 m` output spacing. The
raw local reference costs are precomputed once before iteration.

Two costmap/path snapshots captured from actual M20 MuJoCo runs were replayed
offline. The snapshots stored the live costmap and old `/path`; validation
re-ran weighted A* from the stored start/goal to obtain raw grid paths, then
applied both smoothing implementations to the same input.

| Snapshot | Raw A* turn | Legacy turn | Constrained turn | Legacy length | Constrained length | Max nearest raw-point distance | Runtime |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Planning | 3.93 rad | 1.95 rad | 1.48 rad | 3.52 m | 3.47 m | 0.081 m | 57 ms |
| Detour | 13.35 rad | 3.44 rad | 2.49 rad | 7.54 m | 7.47 m | 0.077 m | 126 ms |

Both constrained outputs passed continuous lethal-cell validation at `0.025 m`
sampling. These results validate the static geometry and costmap contract, not
closed-loop tracking. A restarted MuJoCo run must still confirm controller
behavior, replans, and clearance over time.

![Real M20 costmap replay comparison](assets/m20-constrained-smoothing-real-snapshots.webp)

#### Conclusions From The Replay

1. The improvement is not produced by a long-range shortcut. The constrained
   path remains inside the raw A* local corridor and preserves the same route
   topology while removing grid-scale left/right bends.
2. The constrained smoother outperformed the legacy moving average in both
   recorded cases: cumulative turn fell by about 24% in Planning and 28% in
   Detour, while path length also fell by 0.04 m and 0.07 m respectively.
3. The observed maximum nearest raw-point distance was 0.081 m, below the
   configured 0.10 m displacement limit. Both outputs passed a stricter 0.025 m
   swept-segment lethal-cell replay check.
4. Forty iterations were sufficient for these paths. Measured offline latency
   was 57-126 ms, so increasing the iteration count without evidence would add
   planning delay for little geometric benefit.
5. This evidence validates static path geometry, fallback boundaries, and
   costmap safety checks. It does not yet prove lower closed-loop steering
   oscillation or unchanged clearance during motion; those remain MuJoCo tests.
6. Stage 2 turn-aware A* should remain deferred until runtime testing shows
   that significant raw-path alternation survives the 0.10 m smoothing tube.

#### New Planner Parameters

| Parameter | Current value | Purpose and tuning effect |
| --- | ---: | --- |
| `constrained_path_smoothing_enabled` | `true` | Selects the bounded costmap-aware smoother. `false` restores legacy moving-average behavior. |
| `path_smoothing_iterations` | `40` | Maximum optimizer passes. More may improve convergence but increases planning latency; `0` skips adjustment and only resamples raw A*. |
| `path_smoothing_data_weight` | `0.02` | Pull toward matching raw A* points, range `0..1`. Higher is more route-faithful but retains more grid stair-stepping; lower allows stronger smoothing within the hard tube. |
| `path_smoothing_smoothness_weight` | `0.45` | Neighbor smoothness strength, range `0..0.5`. Higher removes local bends more strongly; `0` disables this correction. |
| `path_smoothing_max_deviation_m` | `0.10 m` | Hard displacement limit from each matching raw A* point. Larger values allow more rounding but weaken route fidelity; `0` disables geometric adjustment. |
| `path_smoothing_collision_sample_spacing_m` | `0.05 m` | Costmap sample interval along candidate segments. Smaller is stricter but costs more CPU; it should not exceed map resolution. |
| `path_smoothing_max_cost_increase` | `2.0` | Maximum allowed mean cost increase over each raw local neighborhood and the uniformly resampled raw whole-path baseline. `0` permits no increase; lethal and out-of-map cells are rejected regardless. |
| `path_smoothing_backtracking_factor` | `0.5` | Fraction of the previous smoothing displacement retained after each failed whole-path validation, range `0..1` exclusive. `0.5` tests half as much adjustment on every retry. |
| `path_smoothing_max_backtracking_steps` | `3` | Number of reduced-fraction retries after the full candidate. With factor `0.5`, three retries evaluate `1.0`, `0.5`, `0.25`, and `0.125`. `0` restores one-shot validation. |
| `path_resample_spacing_m` | `0.10 m` | Final controller waypoint spacing. Smaller better represents curves with more processing; larger reduces point count but can lose tight geometry. |

`publish_raw_path` is a related diagnostic switch rather than a smoothing
coefficient. When enabled it publishes `/raw_path` for Rerun comparison without
changing the path consumed by `LocalPlanner`.

#### Runtime Fallback Observation

A 2026-07-16 MuJoCo stress run issued 121 rapid goal requests. Thirty-four
paths (about 28%) failed final constrained-smoothing validation and correctly
fell back to uniformly resampled raw A*. These fallback paths remain safe but
visibly retain raw grid bends, matching the reported poor-looking cases.

The original warning confirmed fallback but did not record whether validation
found a lethal/out-of-map sample or exceeded the configured mean-cost increase.
Diagnostic commit `a5b45c48` added `reason`, raw/candidate/allowed cost, point
counts, and actual maximum displacement without changing smoothing decisions.

Two subsequent automated sweeps issued 69 goals and captured raw/output paths
for every successful plan:

| Result | Count | Meaning |
| --- | ---: | --- |
| Constrained-smoothed | 36 | Passed collision and final mean-cost validation. |
| Raw-resampled fallback | 12 | Full smoothing candidate was rejected; raw A* topology was published. |
| No path / timeout | 21 | Safe-goal or A* planning did not produce a path; not a smoothing failure. |

All 13 fallback warnings in that process, including one additional manually
issued goal, reported `reason=cost_increase`; none reported `lethal_cell` or
`out_of_bounds`. The scripted candidates exceeded the configured `+2.0`
allowance by 0.031 to 3.157 cost units, with a median excess of 0.53. This shows
that the dominant failure is the all-or-nothing final mean-cost gate, not a
collision-producing optimizer.

For the 36 accepted paths, cumulative turn fell by a median 83.7% and path
length by a median 4.3%. Fallback paths reduced turn by only 34.9% through
uniform resampling and retained visible raw-grid corners. Some accepted paths
still contain large V-shapes already present in raw A*; those are macro route
topology/objective issues outside the 0.10 m local smoothing tube.

![M20 simple-nav goal sweep](assets/m20-smoothing-goal-sweep-2.webp)

#### Implemented Constrained Backtracking

The first backtracking implementation is **global fractional
backtracking**, not local segment replacement. For every raw A* control point
`R[i]` and fully smoothed point `S[i]`, evaluate:

```text
B[i, alpha] = R[i] + alpha * (S[i] - R[i])

alpha = 1.0   full smoothing
alpha = 0.5   half of every smoothing displacement
alpha = 0.25  quarter of every smoothing displacement
alpha = 0.125 one eighth of every smoothing displacement
fallback      uniformly resampled raw A* geometry
```

Try decreasing `alpha` values and publish the largest fraction whose resampled
path passes the unchanged lethal/out-of-map and `raw_cost + 2.0` checks. The
whole-path cost baseline is now the uniformly resampled raw path, so baseline
and candidates use the same waypoint and validation sampling representation.
Start and goal remain fixed, route topology is unchanged, and every reduction
in `alpha` also reduces point displacement from raw A*. Only when all four
nonzero fractions fail does the planner publish the raw-resampled fallback.

The implementation emits one structured record for every rejected fraction
and records `selected_fraction`, baseline/candidate/allowed cost, rejected
fractions, point counts, and actual displacement on acceptance. This makes a
true fallback distinguishable from a straight path that happens to equal raw
resampling.

##### Live Validation Result

The same two 69-goal sweeps were repeated from simulation odometry
`(-1.006, 1.000)` after enabling fractional backtracking. The costmap was
rebuilt by the new process, so structured planner decisions rather than the
sweep script's geometric equality label are authoritative.

| Decision | Count | Share of 49 smoothing decisions |
| --- | ---: | ---: |
| Full smoothing, `alpha=1.0` | 29 | 59.2% |
| Half smoothing, `alpha=0.5` | 14 | 28.6% |
| Quarter smoothing, `alpha=0.25` | 1 | 2.0% |
| Eighth smoothing, `alpha=0.125` | 0 | 0.0% |
| Raw-resampled fallback | 5 | 10.2% |

The immediately preceding run from the same initial odometry produced 37
fallback warnings among 48 successful smoothing decisions, approximately
77.1%. Fractional selection therefore recovered a nonzero smoothed result in
most cases that previously discarded the complete candidate. All 36 rejected
fraction records still reported `cost_increase`; none reported `lethal_cell`
or `out_of_bounds`. Fixed planning-turn and obstacle-detour snapshots selected
`alpha=1.0` and retained their previous geometry and metrics.

This validates the candidate-search structure, but it does not validate the
physical meaning of the fixed `+2.0` mean-cost threshold. That threshold stays
unchanged in this stage for attribution. The next validator iteration should
measure physical clearance and unknown-space exposure while preserving this
fraction schedule.

##### Representative Real-Path Comparison

The following nine panels use paths captured directly from MuJoCo `/raw_path`
and the final controller `/path`. They cover three full-smoothing cases, two
half-smoothing cases, one quarter-smoothing case, one eighth-smoothing case,
and two complete raw-resampled fallbacks.

![M20 fractional backtracking representative paths](assets/m20-fractional-backtracking-typical-paths.webp)

The primary 69-goal run did not select `alpha=0.125`, so an additional
fine-grid sweep sampled 121 goals around the only primary `alpha=0.25` region.
It produced 63 selections at `1.0`, 52 at `0.5`, one at `0.25`, and five at
`0.125`, with no complete fallback. This supplemental run is used only to
provide genuine eighth-fraction geometry; it does not alter the primary
49-decision fallback statistic above.

The `alpha=0.125` example is intentionally close to raw A*: only one eighth of
the full optimizer displacement survives. Complete fallback examples may
still show a lower reported cumulative turn because uniform 0.10 m resampling
removes duplicate grid-cell headings, but their macro A* route is unchanged.

This is deliberately conservative: all path corrections are reduced together,
even if only one area raised the final mean cost. A future local-only repair
could identify violating intervals and reduce smoothing only there, but the
current validator reports a whole-path mean rather than a violating segment.
Local repair would also need continuity checks at both interval boundaries and
another whole-path validation. It should follow the simpler global mechanism
only if test evidence shows that global fractional backtracking removes too
much useful smoothing.

Constrained backtracking addresses the current full-fallback defect. It does
not straighten a large V-shaped raw A* route; turn-aware A* remains the next
separate stage for those macro geometry cases.

## Evidence And Problems

| ID | State | Problem | Evidence | Effect |
| --- | --- | --- | --- | --- |
| P1 | Implemented | A* ranked map cost before distance. | A reproduced route was 0.354 m versus a 0.271 m direct corridor because it saved 8 raw cell-cost units. | Unnecessary winding and endpoint return hooks. |
| P2 | Open / design decision | `initial_safe_radius_meters` is a fixed startup disc around global `(0, 0)`. | `CostMapper._apply_initial_safe_radius` computes distance from world zero, not robot odometry. | It does not clean sparse cells near the robot after movement. |
| P3 | Implemented / runtime validation pending | Spatial-spread stuck detection caused false replans. | Active simulation logs showed forward commands with no obstacle, but 0.20-0.28 m coordinate spread was below the old 1.0 m simulation threshold. | The detector now requires insufficient cumulative path progress instead. |
| P4 | Implemented / runtime validation pending | Grid path topology contains local quantization bends. | Real costmap replay reduced cumulative turn by 24-28% versus legacy smoothing while retaining a bounded A* corridor. | Static geometry improved; closed-loop tracking still requires MuJoCo validation. |
| P5 | Open | Sparse map quality is not separately measured before path planning. | One inspected costmap contained about 38,808 unknown, 39,114 free, and 870 occupied cells. | Planner behavior can vary with local perception coverage rather than only geometry. |
| P6 | Open | Unknown-cell policy differs between planning and local clearance. | A* may cross unknown at a penalty of 80, while local obstacle checking stops only for lethal cost 100. | The robot can plan through unobserved terrain without an explicit risk policy. |

P2, P3, and P5 are related but are not yet proven to have the same root cause.
They must be measured independently before changing control behavior.

## Unknown Cells: Definition And Current Behavior

An occupancy-grid cell is a 2D square of terrain, here normally 0.05 m by
0.05 m. It is not an image pixel. For the current `height_cost` mapper, the
values mean:

| Value | Meaning | Current source |
| --- | --- | --- |
| `-1` | Unknown: the mapper cannot form a sufficiently reliable terrain cost. | No valid point observation remains after filtering/smoothing, or the cell lacks the configured number of valid neighboring observations needed for a slope calculation. |
| `0` | Free / flat enough. | Observed terrain has negligible height change after noise filtering. |
| `1..99` | Traversable but increasingly costly terrain. | The local terrain-height gradient approaches the configured climb limit. |
| `100` | Lethal obstacle / non-traversable after map inflation. | A terrain gradient exceeds the limit or an obstacle footprint has been inflated by robot width. |

Unknown therefore means **"the map does not have enough confidence to label
this square"**, not "there is definitely an obstacle" and not "it is known
free". Sensor blind spots, sparse rays, occlusion, map boundaries, filtered
overhead-only returns, and insufficient neighbor support can all produce it.

The current simple-nav policy is mixed:

- CostMapper and the gradient stage preserve `-1`; obstacle inflation does not
  turn unknown into `100`.
- A* accepts unknown cells and charges `unknown_penalty * cost_threshold`,
  currently `0.8 * 100 = 80`. It will use unknown only when the alternative
  has a higher weighted objective or no route exists.
- A clicked goal that is itself unknown is accepted without the normal safe-goal
  search.
- Local path clearance only stops for cells equal to `100` in the next 3 m of
  the current path. It does not stop solely because a cell is unknown.

This makes unknown a **soft global-planning risk**, but not a local hard stop.
That may be acceptable for a well-tested exploration mode, but it is not yet a
documented safety policy for this platform.

## Exactly When Replanning Happens

Receiving a new goal calls `_plan_path()` immediately. That is a new plan, not
a recovery replan. A new global-costmap message by itself only updates the map
cache; it does **not** automatically create a path.

With an active goal, recovery replanning has these triggers:

| Trigger | Current condition | Result |
| --- | --- | --- |
| Path deviation | Distance from odometry to the published path exceeds `0.9 m`. | Full remaining route is regenerated. |
| No progress / stuck | Only in `path_following`: less than `0.2 m` of maximum cumulative path progress for 8 s, while forward command exceeds `0.1 m/s` and goal distance exceeds `0.5 m`. | Full remaining route is regenerated. |
| Obstacle ahead | The local controller finds a lethal (`100`) cell in the next 3 m of its path footprint. | It stops, then requests a full remaining route. |
| Local planner error | The local controller thread raises an exception. | It stops, then requests a full remaining route. |

Arrival is not a replan: it clears the goal. Before a recovery replan starts,
the planner also has safeguards: if the robot is already within `0.5 m` of the
goal it declares arrival; replanning can be disabled; and `ReplanLimiter`
allows at most six attempts while the robot remains within 2 m of its first
retry position. After the limit it cancels the goal instead of retrying
forever.

The previous stuck calculation measured the *spread* of positions in the last
8 seconds, not distance advanced along the intended path. The replacement
projects odometry onto the active path, retains the maximum reached path
distance, and resets its timer only after another `0.2 m` of cumulative
progress. Initial/final rotation never enters the condition because it is not
in `path_following`.

### Stuck-Replan Diagnostic Record

The stuck log now emits one structured record immediately before each recovery
replan. It contains:

- `path_progress_m`, `path_length_m`, `path_remaining_m`, and direct
  `goal_distance_m` projected from current odometry;
- latest published `cmd_linear_x_m_s`, `cmd_linear_y_m_s`, and
  `cmd_angular_z_rad_s`;
- odometry position/yaw and LocalPlanner state;
- `obstacle_ahead` from the local 3 m lethal-obstacle check; and
- `path_progress_delta_m`, the time window, and the thresholds used by the
  path-progress condition.

This is diagnostic instrumentation only. It does not alter replan thresholds
or controller behavior. A restarted process is required for the new fields to
appear in logs.

## What A "Replan" Replaces

The current stack does not maintain a separate global route plus a local
segment planner. On every recovery replan it:

1. Stops the local controller and publishes an empty path.
2. Uses the current odometry as the new start and retains the original active
   goal.
3. Finds a safe goal, regenerates the whole navigation costmap, and runs A*
   from the current position to that goal.
4. Smooths/resamples the complete remaining geometric path, publishes it, and
   starts a new LocalPlanner controller thread.

So it is a **full remaining global-path replan**, not a repair of only the next
few meters. It is also not a time-parameterized trajectory optimization: the
result is a complete 2D geometric path; the LocalPlanner produces velocity
commands online at 10 Hz while following it.

## Candidate Solutions

### A. Balance Length And Map Cost

Use a scalar objective per A* edge:

```text
objective += distance_weight * move_distance
           + cell_cost_weight * (cell_cost / cost_threshold)
```

The cell cost is normalized by the obstacle threshold, so the two terms can be
tuned at similar numerical scales. The A* heuristic contains only the
non-negative distance component and remains admissible.

**Benefits:** directly prevents a small clearance-cost saving from justifying a
large geometric detour; works for both global and near-goal maps.

**Trade-off:** excessive distance weighting can cut too close to obstacles.
The map-cost term must remain nonzero and tests must include narrow corridors.

### B. Constrained Local Geometry Smoothing

After A* finds a route, iteratively reduce local second differences while
keeping every point close to its corresponding raw A* point. Accept a move only
when adjacent swept segments remain valid and their effective cost does not
materially exceed the raw local subpath.

**Benefits:** removes local grid stair-stepping without replacing the global
route or discarding the cost-field preference selected by A*.

**Trade-off:** the displacement bound limits how much curvature can be removed.
If raw A* direction changes exceed that envelope, Step 2 turn-aware A* becomes
necessary.

An earlier unconstrained visibility-shortcut prototype was rejected because a
collision-free straight segment could still cut through a high-cost or sparse
corridor and erase A* route structure. Do not reintroduce farthest-visible
point connection as the smoothing policy.

### C. Split Startup Clearing From A Deliberate Dynamic Local-Map Policy

Keep `initial_safe_radius_meters` as a startup-only mechanism, because that is
what its name and current implementation describe. If dynamic near-robot
cleanup is required, introduce a separately named and explicitly bounded
policy that uses current odometry and reports its age. It must define whether
it clears only sensor self-points, only a robot footprint, or all unknown
cells, and what happens when odometry is stale.

**Benefits:** separates a startup convenience from an ongoing perception and
safety policy, avoiding accidental use of a fixed map-origin disc as a dynamic
robot bubble.

**Trade-off:** indiscriminately clearing near-robot unknown cells can hide a
real obstacle if the radius is too large or odometry is wrong. It requires a
safety review and map-level tests before real-robot use.

### D. Upgrade Replan Progress And Stability Logic

The first implementation replaces spatial spread with cumulative path progress:
only `path_following` can be stuck; progress of at least `0.2 m` refreshes the
8-second timer; forward command must exceed `0.1 m/s`; and goals within
`0.5 m` are excluded. Obstacle and path-deviation replans remain independent.

Future work can add slow-progress warnings, odometry freshness checks, and a
minimum replan interval after this simpler condition is validated.

**Benefits:** distinguishes an actual blockage from slow valid motion or map
jitter, reducing route churn without suppressing genuine obstacle recovery.

**Trade-off:** this adds observability first; it should not be "fixed" by only
increasing the timeout.

### E. Define An Explicit Unknown-Cell Policy And Map-Quality Metrics

Expose local unknown-cell ratio, free-cell ratio, and point-cloud freshness
around the robot, path corridor, and requested goal. Then choose and test an
operating mode: unknown allowed with penalty, unknown prohibited near the
robot, or unknown prohibited everywhere except explicitly selected exploration
goals. The global planner, safe-goal selection, and local clearance check must
follow the same documented contract.

**Benefits:** gives an operator and tests a clear distinction between an
algorithmic failure and insufficient perception.

**Trade-off:** requires selecting platform- and sensor-specific thresholds and
may reduce reachable area in sparse maps.

#### Strategy 1: Safe Navigation (Recommended Default)

Use this for routine navigation, testing near people, and any repeatable route
where exploration is not the goal. Unknown is treated as non-traversable:

- A* does not cross unknown cells.
- Goals must resolve to an observed safe cell.
- Local clearance stops if unknown enters the imminent path footprint.
- The robot waits for perception/map updates or asks for a new known-safe goal.

This sacrifices reachability in sparse maps for an unambiguous safety rule. It
is the most suitable default for the current platform because local clearance
otherwise permits unobserved terrain.

#### Strategy 2: Cautious Exploration

Use this only when mapping/exploration is explicitly requested. Unknown stays
traversable at a high global cost, but the permission is bounded:

- Only frontier goals or an explicitly marked exploration goal may enter an
  unknown corridor.
- Unknown traversal uses a reduced speed and a short forward horizon.
- The robot stops when fresh perception does not convert the approaching
  unknown footprint into observed free space before the horizon is consumed.
- The log records mode, unknown-path length, map age, and the stop reason.

This lets a robot expand a map without pretending unknown is free. It requires
a dedicated exploration-mode contract and tests before real-robot use.

Do not use a moving `initial_safe_radius_meters` disc as either strategy. It
would erase evidence instead of deciding how to handle uncertainty.

### F. Separate Global Route From Local Trajectory Recovery

Keep the weighted A* route as the global geometric plan, but introduce a local
rolling-horizon trajectory or path optimizer for short-range obstacle response
and speed control. Major map topology changes should re-run global A*; small
local changes should be resolved locally when safe.

**Benefits:** avoids rebuilding the entire route for a transient local map
change and matches the longer-term architecture of global path planner ->
speed optimizer -> tracking controller.

**Trade-off:** this is an architectural upgrade requiring clear module
contracts, local-map inputs, and safety tests. It should follow observability
and unknown-policy work, not precede them.

## Implemented Upgrade: P1 Weighted A*

Commit `9726f754` implements candidate A for the simple-nav planner chain.

| Layer | Change |
| --- | --- |
| `min_cost_astar.py` | Added `distance_weight` and `cell_cost_weight`; Python fallback applies the weighted objective. |
| `min_cost_astar_cpp.cpp` | Applied the same objective, priority ordering, validation, and admissible distance heuristic in the native implementation. |
| `ReplanningAStarPlannerConfig` | Added validated non-negative settings `path_length_weight=1.0` and `path_cell_cost_weight=3.0`. |
| `GlobalPlanner` | Passes these settings only into its `min_cost_astar` call. |
| Regression test | Verifies legacy weights choose a low-cost terminal hook while the simple-nav weights choose the direct monotonic route, in both implementations. |
| Stuck monitor | Replaces spatial spread with cumulative active-path progress and adds condition coverage. |

### Compatibility Boundary

`min_cost_astar` itself retains defaults `distance_weight=0.0` and
`cell_cost_weight=1.0`. Existing callers therefore preserve the old
cost-first behavior unless they explicitly opt in. The true simple-nav
`ReplanningAStarPlanner` is the only path enabled with the new defaults.

### Verification Completed

- Native C++ extension rebuilt successfully in the VM.
- New C++/Python terminal-hook regression: 2 passed.
- `m20-true-simple-nav-sim` blueprint tests and blueprint-registry test: 5
  passed.
- Lint and formatting checks for the A* directory passed.
- The complete historical A* test file remains blocked by protected LFS test
  data unavailable to this VM; the new regression has no LFS dependency.

## Next Validation Sequence

1. Restart `m20-true-simple-nav-sim`; the already-running process cannot load
   the new Python module or shared library.
2. Reproduce the former near-goal case and capture raw A* cells and the
   resampled path. Confirm the endpoint no longer overshoots and returns.
3. Test an open route, an obstacle-constrained route, and a narrow passage.
   Compare path length, minimum clearance, completion rate, and replans.
4. If necessary, tune `path_length_weight` and `path_cell_cost_weight` from
   the current `1.0` and `3.0` defaults. Do not tune both together without
   recording the scenario and result.
5. Restart and validate the P3 path-progress monitor before tuning any more
   A* weights. Then investigate P2/P5/P6 separately.
6. Inspect raw A* after constrained smoothing is validated. Add direction-state
   turn cost only if material alternating turns remain outside the smoothing
   envelope; do not reintroduce visibility shortcutting.

## Acceptance Criteria For This Phase

- In a clear, traversable corridor, the output route does not intentionally
  overshoot an attainable final grid cell and return to it.
- A shorter safe route wins over a materially longer route that only saves a
  small gradient cost.
- Obstacle clearance remains governed by the map-cost component and robot
  footprint expansion; route shortening must not cross occupied or
  unsafe-clearance cells.
- Python fallback and native C++ implementation return equivalent routes for
  regression scenarios.
- Stuck replans are observed and diagnosed separately, not hidden by A*
  weight tuning.

## Related Records

- `docs/codex-sessions/completed/2026-07-15_2304_m20-true-simple-nav-path-quality-investigation.md`
- `docs/development/issues/m20-goal-pose-final-orientation-contract.md`
- `dimos/mapping/costmapper.py`
- `dimos/navigation/replanning_a_star/`
