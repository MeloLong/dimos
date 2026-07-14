# Issue: M20 Dan Navigation Problems After Goal Replacement

- Status: Open
- Overall priority: High
- Scope: M20 Dan navigation goal replacement, path commit gating, and runtime observability
- Affected blueprint: `m20-dan-nav-sim`
- Affected modules: `DanLocalPlanner`, `DanHolonomicTC`, Rerun visualization
- Discovered on: 2026-07-14

## Overall Assessment

This issue contains four related problems. They do not all have the same risk:

| No. | Problem | Impact | Potential hazard |
| --- | --- | --- | --- |
| 1 | Stale path can be committed after a new goal | High | Robot may follow the previous route instead of the latest command |
| 2 | Tracking control has no explicit goal/path consistency guard | High | A stale route can enter motion control without being rejected downstream |
| 3 | Rerun can retain and display an old path | Medium | Operator may believe the visible route belongs to the latest goal |
| 4 | Logs and tests do not expose or cover the failure boundary | Medium | The defect can recur and be difficult to diagnose before real-robot testing |

Problems 1 and 2 affect navigation behavior and must be resolved before
relying on repeated-goal testing. Problem 3 is primarily visualization, but it
can conceal problems 1 and 2. Problem 4 is an engineering and safety-net gap.

## Evidence From Simulation

The run log
`logs/20260714-103638-m20-dan-nav-sim/main.jsonl` shows:

- `10:44:33` local time: path following started;
- `10:45:09`: first goal reached;
- `10:45:16`: a second path following cycle started;
- `10:46:17`: second goal reached;
- `10:46:44`: another path following cycle started;
- `10:50:28`: the follower stopped without another `Reached goal position` event.

The log contains no `no path between start and goal` warning or planning error.
The third cycle is therefore not proven to be an MLS unreachable-goal result.
It is consistent with a non-empty stale path being accepted and then failing to
reach the newly clicked target.

## 1. Stale Path Can Be Committed After a New Goal

**Severity: High. Functional navigation defect.**

When a new goal is armed, `_ReplanGate.on_planner_path` first checks whether the
path endpoint matches the new goal. If it does not match, the code falls
through to the independent `lock_replan` release check. Once the lock is
released, the stale path is committed anyway.

The simulation config uses `lock_replan=1.0` in
`dimos/robot/deeprobotics/m20/nav/m20_dan_nav.py`.

### Impact

- A new click can appear to be accepted while the previous route is still
  being followed.
- The robot can enter `path_following` without the committed path ending at the
  latest goal.
- The robot may stop at the old goal, move in an unexpected direction, or fail
  to converge to the new goal.

### Potential Hazard

On a real robot, the vehicle could continue moving toward a previously selected
location after the operator believes a new command has replaced it. Near
obstacles, people, or platform edges, this is a motion-safety risk rather than
only a navigation-quality issue.

### Reproduction

1. Create a gate with `lock_replan=0.5`.
2. Commit a path ending at `(5.0, 0.0)`.
3. Move the robot to `(1.0, 0.0)`, releasing the lock.
4. Arm a new goal at `(9.0, 0.0)`.
5. Submit a stale path ending at `(5.0, 0.0)`.

Observed result with the current implementation:

```text
forwarded=True
armed_goal=[9.0, 0.0]
```

The stale path is forwarded while the new goal remains armed.

## 2. DanHolonomicTC Has No Explicit Goal/Path Consistency Guard

**Severity: High. Missing downstream safety invariant.**

`DanHolonomicTC` receives a `Path` but no goal identity, goal revision, or
validated relationship between the path endpoint and the latest clicked goal.
When a non-empty path arrives, it starts or updates tracking. The controller
therefore relies entirely on `DanLocalPlanner` to enforce goal consistency.

### Impact

- A gate mistake is passed directly into the motion controller.
- The controller cannot distinguish a valid replan from an old in-flight path.
- Future planners or adapters can reintroduce the same failure if they publish
  paths without preserving goal identity.

### Potential Hazard

This removes a defense-in-depth layer from the command chain. A stale path can
be converted into velocity commands even though it does not correspond to the
operator's latest goal. The immediate hazard is unintended motion; the longer
term hazard is that the system contract is not safe across module boundaries.

### Expected Direction

The short-term fix belongs in `DanLocalPlanner`: while a fresh goal is armed,
reject every non-empty path whose endpoint is outside
`goal_commit_tolerance_m`, regardless of lock state. A stronger architecture
should also carry a goal revision or validated endpoint through the planner to
the tracking controller.

## 3. Rerun Can Display an Old Path After a Stop or Empty Plan

**Severity: Medium. Primarily visualization and operator awareness.**

`m20_dan_nav.py` returns `None` from the Rerun visual override when the path is
empty. The documented intent is to keep the last displayed route visible.

### Impact

- The green route in Rerun may not represent the current planner output.
- An empty safety-stop path can leave an old route visible on screen.
- Operators may interpret stale visualization as proof that the latest goal has
  a valid route.

### Potential Hazard

This does not itself issue motion commands, so it is not the primary cause of
unintended movement. Its hazard is diagnostic: it can delay recognition of
problem 1 or 2 and cause an operator to make decisions based on an obsolete
route.

### Expected Direction

Separate the displayed path state from the active controller path. The viewer
should clear or explicitly mark a stopped/invalid route, while retaining old
paths only in a visibly historical layer.

## 4. Runtime Logs and Tests Do Not Cover the Failure Boundary

**Severity: Medium. Verification and maintainability gap.**

The existing test
`test_fresh_click_does_not_commit_a_stale_replan` only checks a stale path while
the replan lock is still held. It does not test a fresh goal after the lock has
already been released.

The runtime logs also omit the clicked goal coordinates, path endpoint,
goal/path revision, and gate commit reason. The log can show that tracking
started, but not why that path was accepted.

### Impact

- The current six local-planner tests pass while the released-lock defect is
  still present.
- A future refactor can reintroduce the behavior without a failing test.
- Field reports cannot easily distinguish no-path planning failure, stale-path
  forwarding, controller non-convergence, and stale visualization.

### Potential Hazard

The main risk is delayed detection. A defect can appear to pass simulation and
reach hardware testing without a clear safety boundary or forensic evidence.
This increases the chance that incorrect motion is discovered only during
interactive operation.

### Required Test and Diagnostic Coverage

- Add a regression test for a released lock followed by a stale path.
- Keep coverage for cold start, in-lock stale paths, goal replacement,
  cancellation, and empty-path safety stops.
- Log the goal revision or coordinates, candidate path endpoint, and commit
  reason (`fresh_goal`, `lock_release`, `cold_start`, or `empty_stop`).

## Required Resolution

1. While a fresh goal is armed, suppress every non-empty path whose endpoint is
   outside `goal_commit_tolerance_m`.
2. Do not allow lock release to override a pending goal-match requirement.
3. Preserve the empty-path safety behavior: forward it immediately, drop the
   committed path, and stop the controller.
4. Add goal/path identity or revision information at the module boundary where
   practical.
5. Make stopped or invalid routes distinguishable from the active Rerun path.
6. Add regression tests and structured diagnostics for each commit reason.

## Relevant Files

- `dimos/navigation/dannav/local_planner/module.py`
- `dimos/navigation/dannav/local_planner/test_dan_local_planner.py`
- `dimos/navigation/dannav/holonomic_tc/module.py`
- `dimos/robot/deeprobotics/m20/nav/m20_dan_nav.py`
- `logs/20260714-103638-m20-dan-nav-sim/main.jsonl`

## Execution Plan

The four problems must be addressed separately. Functional motion correctness
comes before visualization polish. Do not use the real robot for repeated-goal
validation until steps 1 and 2 have passed their tests.

### Step 1: Add the Regression Tests First

Extend
`dimos/navigation/dannav/local_planner/test_dan_local_planner.py` before
changing the gate implementation.

Required cases:

1. A fresh goal with a matching path commits once and consumes the armed goal.
2. A fresh goal with a stale path is suppressed while the lock is held.
3. A fresh goal with a stale path is also suppressed after `lock_replan` is
   released.
4. A fresh goal at cold start cannot accept a non-matching path.
5. An empty path is still forwarded immediately and resets the committed path.
6. Cancellation, cold start, and normal lock-based replanning retain their
   existing behavior.

The released-lock test is the key regression test. It must fail against the
current implementation and pass after the gate fix.

### Step 2: Fix DanLocalPlanner Goal Gating

Update
`dimos/navigation/dannav/local_planner/module.py` so the decision order is:

```text
empty path
  -> forward immediately and stop
pending fresh goal
  -> matching endpoint: commit and disarm
  -> non-matching endpoint: suppress
no pending goal + no committed path
  -> cold-start commit
no pending goal + lock released
  -> commit replan
otherwise
  -> suppress
```

The lock-release branch must never override a pending goal-match requirement.
Run the local-planner unit tests and the focused M20 navigation tests after
this change.

Suggested commit boundary:

```text
test(m20): cover stale path after released replan lock
fix(m20): reject stale path while new goal is pending
```

### Step 3: Add a Controller-Side Consistency Guard

Update `dimos/navigation/dannav/holonomic_tc/module.py` after Step 2 is stable.
The controller currently receives a path without the identity of the goal that
created it. Add a short-term guard by providing the current goal to the
controller and rejecting a path whose endpoint is outside the configured goal
tolerance. Rejected or invalid paths must publish zero velocity.

The longer-term contract should carry a goal revision or goal ID with the path
so that planners and controllers can distinguish a new plan from an old
in-flight message. Validate this contract with tests for matching and stale
revisions.

Suggested commit boundary:

```text
fix(m20): add goal consistency guard to tracking controller
```

### Step 4: Make Active and Historical Rerun Paths Distinct

Update `dimos/robot/deeprobotics/m20/nav/m20_dan_nav.py` after motion behavior
is correct.

- Clear the active route when an empty or invalid path is received.
- Keep historical routes in a separate, explicitly historical entity if review
  is needed.
- Publish a visible active-path status such as `active`, `replanning`,
  `stopped`, or `invalid`.
- Verify that a stopped planner does not leave an old route looking active in
  Rerun.

This step must not change the planner or controller's motion decision.

Suggested commit boundary:

```text
fix(m20): clear inactive route from rerun viewer
```

### Step 5: Add Runtime Diagnostics and End-to-End Verification

Add structured logs around path commit decisions. Each decision should expose:

```text
goal_revision or goal_position
candidate_path_endpoint
accepted
commit_reason: cold_start / fresh_goal / lock_release / empty_stop
```

Then repeat the simulation scenario:

1. Send goal A and verify the committed endpoint matches A.
2. Move the robot to the table edge or underneath it.
3. Send goal B and verify that old paths are suppressed until a path ending at
   B arrives.
4. Verify that the controller either reaches B or stops with an explicit
   failure state; it must not silently follow A's path.
5. Send goal C after arrival and repeat the same checks.
6. Confirm the Rerun active route and logs agree with the latest goal.

Record the run log path, goal coordinates, path endpoints, arrival status, and
any stop reason in the test report. Do not treat a visible green line alone as
evidence of a valid current plan.

Suggested final documentation boundary:

```text
docs(m20): record goal replacement verification
```

## Completion Gate

This issue is complete only when all of the following are true:

- stale paths are rejected during and after a released replan lock;
- cold-start and empty-path safety behavior remains correct;
- the tracking controller has a goal/path consistency guard or an equivalent
  versioned path contract;
- Rerun clearly distinguishes an active path from historical or stopped paths;
- focused unit tests pass;
- the repeated-goal MuJoCo scenario passes with structured logs;
- the final commits are pushed to the active development branch.
