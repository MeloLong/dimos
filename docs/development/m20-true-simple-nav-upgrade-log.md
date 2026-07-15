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
| P2 | Open | `initial_safe_radius_meters` clears only around global `(0, 0)`. | `CostMapper._apply_initial_safe_radius` computes distance from world zero, not robot odometry. | Near-robot sparse or unknown cells can remain after the robot moves away from origin. |
| P3 | Open | Repeated stuck detection causes replanning. | Active simulation logs reported `Robot is stuck. Replanning.` about every 8 seconds. | Path is replaced frequently, making routes look unstable and complicating diagnosis. |
| P4 | Candidate | Grid path topology is passed directly to smoothing. | Smoothing made the existing raw A* terminal correction visually obvious; it did not create it. | Even a valid grid route can contain unnecessary bends. |
| P5 | Open | Sparse map quality is not separately measured before path planning. | One inspected costmap contained about 38,808 unknown, 39,114 free, and 870 occupied cells. | Planner behavior can vary with local perception coverage rather than only geometry. |

P2, P3, and P5 are related but are not yet proven to have the same root cause.
They must be measured independently before changing control behavior.

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

### C. Make The Safe Radius Follow Robot Odometry

Pass current odometry or robot position into CostMapper and clear the initial
safe disc around that current pose, rather than around global zero. Define the
behavior explicitly for missing/stale odometry and for global-map updates.

**Benefits:** removes local sensor sparsity artefacts where the robot currently
stands.

**Trade-off:** clearing cells around the vehicle can hide a real obstacle if
the radius is too large or odometry is wrong. It requires a safety review and
map-level tests before real-robot use.

### D. Diagnose Stuck Replanning Separately

Record the active path revision, odometry displacement, controller commands,
and local obstacle state when the 8-second tracker fires. Check whether the
robot was actually blocked, making insufficient progress, or was following a
newly replaced path.

**Benefits:** avoids treating a state-estimation, controller, or goal-tolerance
problem as a path-search problem.

**Trade-off:** this adds observability first; it should not be "fixed" by only
increasing the timeout.

### E. Establish Map-Quality Admission Metrics

Expose local unknown-cell ratio, free-cell ratio, and point-cloud freshness
around the robot and the requested goal. Reject or label plans when the map is
too sparse for the configured safety envelope.

**Benefits:** gives an operator and tests a clear distinction between an
algorithmic failure and insufficient perception.

**Trade-off:** requires selecting platform- and sensor-specific thresholds.

## Implemented Upgrade: P1 Weighted A*

Commit `9726f754` implements candidate A for the simple-nav planner chain.

| Layer | Change |
| --- | --- |
| `min_cost_astar.py` | Added `distance_weight` and `cell_cost_weight`; Python fallback applies the weighted objective. |
| `min_cost_astar_cpp.cpp` | Applied the same objective, priority ordering, validation, and admissible distance heuristic in the native implementation. |
| `ReplanningAStarPlannerConfig` | Added validated non-negative settings `path_length_weight=1.0` and `path_cell_cost_weight=3.0`. |
| `GlobalPlanner` | Passes these settings only into its `min_cost_astar` call. |
| Regression test | Verifies legacy weights choose a low-cost terminal hook while the simple-nav weights choose the direct monotonic route, in both implementations. |

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
5. Instrument and fix P2/P3/P5 as separate work items before attributing their
   effects to the new objective.
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
