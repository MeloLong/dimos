# Issue: M20 Configuration Help Must Not Imply Hot Reload

- Status: Open
- Priority: Medium
- Scope: M20 simulation configuration observability and operator workflow
- Affected command: `dimos run m20-dan-nav-sim --help`
- Discovered on: 2026-07-14

## Finding

Editing `dimos/robot/deeprobotics/m20/config/mujoco_sim.yaml` and then running
`dimos run m20-dan-nav-sim --help` shows the edited values. This does not prove
that an already-running simulation has applied the change.

The help command starts a new CLI process. That process imports the M20
blueprint, reads the YAML, resolves the module arguments, and exits after
printing the help text. The existing simulation process keeps the values that
were resolved when its modules were created.

This creates a misleading hot-reload signal: the operator sees the new value
in a second shell while the running MLS planner and MuJoCo adapter continue to
use the old value.

## Current Behavior

The effective lifecycle is:

```text
edit YAML
  -> new `dimos ... --help` process reads new YAML
  -> help output shows new values
  -> existing run remains unchanged
```

The running process currently has no file watcher, configuration revision
signal, or runtime configuration RPC for these profile values. YAML edits are
therefore restart-only changes.

To apply an edited profile, restart the simulation:

```bash
dimos stop
dimos --rerun-open none run m20-dan-nav-sim
```

No native rebuild is needed for a YAML-only change. The active values should
be verified from the startup log or the running `mls_planner` process after the
restart, rather than from a separate `--help` invocation.

## Reproduction

1. Start `m20-dan-nav-sim` with a known `robot_height` or
   `wall_clearance_m` value.
2. Edit that value in `mujoco_sim.yaml` without stopping the run.
3. Run `dimos run m20-dan-nav-sim --help` in another shell.
4. Observe the new value in the help output.
5. Inspect the existing native planner command or behavior and observe that
   it still uses the value captured at startup.
6. Restart the simulation and confirm that the new value is then applied.

## Impact

- Operators can believe they are testing a new planner envelope while the
  running planner still uses stale geometry or clearance values.
- A viewer can remain connected during the edit, masking the fact that the
  backend was not reconfigured.
- Automated tuning or deployment scripts may incorrectly treat `--help` as a
  runtime configuration validation mechanism.

## Short-Term Resolution

Keep the current restart-only semantics, but make them explicit and
observable:

- document that YAML changes require a full run restart;
- make help or startup output state that it reports a newly resolved process,
  not the active run;
- log the selected profile path, content hash or revision, and key resolved
  MLS values at startup;
- expose the active configuration revision and resolved values through a
  status or diagnostic command;
- optionally warn when the profile file changes while a run is active.

## Future Hot-Reload Requirements

If live tuning is needed, it must be an explicit controlled feature rather
than a side effect of editing a file. The implementation must:

- validate a versioned configuration update before applying it;
- define which fields are safe to update live and which require restart;
- update Python and native modules through a coordinated runtime channel;
- apply updates atomically and reject invalid or unsafe values without a
  partial update;
- report the active revision and update result to the operator;
- retain the active revision in the run log for later replay and diagnosis.

Geometry, collision margins, and safety limits require particular care because
an unnoticed partial update could make planning and control disagree.

## Acceptance Criteria

- Documentation and command help clearly state that YAML edits are
  restart-only until a real reload mechanism exists.
- The active run exposes the profile path, revision or hash, and resolved key
  values.
- `--help` output cannot reasonably be interpreted as proof that an existing
  run was reconfigured.
- Tests cover the distinction between a new CLI process and the existing run.
- If hot reload is later implemented, invalid and unsafe updates are rejected
  atomically and the active revision is observable.

## Related Issue

`docs/development/issues/mujoco-yaml-config-reload.md` records the underlying
implementation gap: the current YAML loader is startup-only. This issue tracks
the separate operator-facing observability and workflow problem.

## Relevant Files

- `dimos/robot/deeprobotics/m20/config/mujoco_sim.yaml`
- `dimos/robot/deeprobotics/m20/nav/m20_dan_nav.py`
- `dimos/robot/cli/dimos.py`
- `docs/development/issues/mujoco-yaml-config-reload.md`
