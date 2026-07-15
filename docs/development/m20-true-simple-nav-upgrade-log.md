# M20 True Simple-Nav Upgrade Log

- Status: Active engineering record
- Baseline: `codex/wd-m20-mujoco-sim`, commit `9726f754`
- Scope: MuJoCo `m20-true-simple-nav-sim` planning path quality and stability
- Updated: 2026-07-15

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
  -> smooth_resample_path (0.1 m)
  -> LocalPlanner tracking controller
  -> nav_cmd_vel
```

The global planner uses a Voronoi-derived costmap for longer goals and a
regular gradient costmap nearer the goal. Low map cost generally means more
clearance from obstacles, but it is not a direct measure of geometric path
length.

## Evidence And Problems

| ID | State | Problem | Evidence | Effect |
| --- | --- | --- | --- | --- |
| P1 | Implemented | A* ranked map cost before distance. | A reproduced route was 0.354 m versus a 0.271 m direct corridor because it saved 8 raw cell-cost units. | Unnecessary winding and endpoint return hooks. |
| P2 | Open / design decision | `initial_safe_radius_meters` is a fixed startup disc around global `(0, 0)`. | `CostMapper._apply_initial_safe_radius` computes distance from world zero, not robot odometry. | It does not clean sparse cells near the robot after movement. |
| P3 | Implemented / runtime validation pending | Spatial-spread stuck detection caused false replans. | Active simulation logs showed forward commands with no obstacle, but 0.20-0.28 m coordinate spread was below the old 1.0 m simulation threshold. | The detector now requires insufficient cumulative path progress instead. |
| P4 | Candidate | Grid path topology is passed directly to smoothing. | Smoothing made the existing raw A* terminal correction visually obvious; it did not create it. | Even a valid grid route can contain unnecessary bends. |
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

### B. Visibility Shortcutting Or Theta-Star

After A* finds a collision-free grid route, attempt to replace consecutive
segments with a direct line only when every crossed cell respects occupancy and
clearance constraints.

**Benefits:** removes grid stair-stepping and residual bends even if the cost
field is irregular.

**Trade-off:** requires robust line traversal and clearance validation. It is a
second-stage improvement, not a substitute for correcting the objective.

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
6. Consider visibility shortcutting only after the weighted objective and map
   quality have been evaluated.

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
