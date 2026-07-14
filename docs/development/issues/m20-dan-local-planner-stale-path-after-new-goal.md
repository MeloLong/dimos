# Issue: DanLocalPlanner Can Forward a Stale Path After a New Goal

- Status: Open
- Priority: High
- Scope: M20 Dan navigation goal replacement and path commit gating
- Affected blueprint: `m20-dan-nav-sim`
- Affected module: `DanLocalPlanner`
- Discovered on: 2026-07-14

## Finding

After the robot reaches one goal, a new clicked goal can cause
`DanLocalPlanner` to forward a path whose endpoint still belongs to the
previous goal. This occurs when the replan lock has already been released.

The downstream `DanHolonomicTC` then enters `path_following`, even though the
forwarded path is not confirmed against the newly clicked goal. The robot can
continue tracking an old or stale route and fail to converge to the new goal.

## Evidence From Simulation

The run log
`logs/20260714-103638-m20-dan-nav-sim/main.jsonl` shows:

- `10:44:33` local time: path following started;
- `10:45:09`: first goal reached;
- `10:45:16`: a second path following cycle started;
- `10:46:17`: second goal reached;
- `10:46:44`: another path following cycle started;
- `10:50:28`: the follower stopped without another `Reached goal position` event.

The same log contains no `no path between start and goal` warning or planning
error. This means the third failure is not proven to be an MLS unreachable-goal
result. It is consistent with a non-empty stale path being accepted and then
not reaching the newly clicked target.

## Root Cause

`_ReplanGate.on_planner_path` checks a fresh clicked goal first:

```text
fresh goal + path endpoint within tolerance -> commit and disarm
```

However, when the endpoint does not match the fresh goal, execution falls
through to the independent lock-release check:

```text
lock released -> commit path
```

There is no unconditional rejection while `_armed_goal` is still waiting for a
matching path. Therefore a stale path can pass through whenever the robot has
advanced at least `lock_replan` metres on the previous committed path.

The simulation config uses `lock_replan=1.0` in
`dimos/robot/deeprobotics/m20/nav/m20_dan_nav.py`.

## Minimal Reproduction

The following state reproduces the defect without running MuJoCo:

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

The stale path is forwarded while the new goal remains armed. The existing
test `test_fresh_click_does_not_commit_a_stale_replan` only exercises the case
where the lock is still held, so all current local-planner tests pass while
this released-lock case remains uncovered.

## Impact

- A new goal can appear to be accepted while the controller is still following
  the previous route.
- The viewer can show a valid-looking green path that does not correspond to
  the latest click.
- The controller may remain in `path_following` without reaching the intended
  goal, especially after the robot has stopped at a previous goal or near an
  obstacle.
- Debugging is difficult because the current logs do not include goal IDs,
  path endpoint coordinates, or the gate's commit reason.

## Expected Resolution

While a fresh goal is armed, the gate must suppress every non-empty planner path
whose endpoint is outside `goal_commit_tolerance_m`, regardless of whether
`lock_replan` has elapsed. The lock-release rule should only be considered once
there is no pending fresh goal.

The empty-path safety behavior must remain unchanged: an empty path is forwarded
immediately, the committed path is dropped, and the controller is stopped.

## Acceptance Criteria

- A fresh goal cannot commit a path ending outside
  `goal_commit_tolerance_m`, even when `lock_replan` is already released.
- A matching path commits once and consumes the armed goal.
- Stale paths remain suppressed until a matching path or an empty safety-stop
  path arrives.
- A regression test covers a released lock followed by a stale path.
- Tests retain the current behavior for cold start, in-lock stale paths, goal
  replacement, cancellation, and empty-path safety stops.
- Runtime diagnostics expose at least the clicked goal, committed path endpoint,
  and whether a path was committed because of a fresh goal or lock release.

## Related Observability Issue

`m20_dan_nav.py` currently returns `None` for empty paths in the Rerun visual
override, intentionally retaining the last displayed route. This can make a
stale path remain visible after a planner stop and should be considered when
validating the fix, although it is a separate visualization concern.

## Relevant Files

- `dimos/navigation/dannav/local_planner/module.py`
- `dimos/navigation/dannav/local_planner/test_dan_local_planner.py`
- `dimos/robot/deeprobotics/m20/nav/m20_dan_nav.py`
- `logs/20260714-103638-m20-dan-nav-sim/main.jsonl`
