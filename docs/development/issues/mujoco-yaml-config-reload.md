# Issue: MuJoCo YAML Changes Are Not Hot-Reloaded

- Status: Open
- Priority: Medium
- Scope: M20 MuJoCo simulation configuration lifecycle
- Affected command: `dimos --rerun-open none run m20-dan-nav-sim`
- Discovered on: 2026-07-14

## Problem

Editing `dimos/robot/deeprobotics/m20/config/mujoco_sim.yaml` appears to take
effect immediately when the operator runs:

```bash
dimos run m20-dan-nav-sim --help
```

This is misleading. `--help` starts a new CLI process, imports the M20
blueprint, reads the YAML, validates it, and prints the newly resolved values.
It does not update an already-running simulation.

An existing `m20-dan-nav-sim` process keeps the configuration captured when its
modules were created. The MLS native process receives its values at startup,
and the MuJoCo connection creates its sensor configuration at startup. Neither
path currently watches the YAML file or subscribes to a runtime configuration
update stream.

## Reproduction

1. Start the simulation:

   ```bash
   dimos --rerun-open none run m20-dan-nav-sim
   ```

2. Change `robot_height` or `wall_clearance_m` in
   `dimos/robot/deeprobotics/m20/config/mujoco_sim.yaml`.
3. Observe that the existing MLS process arguments do not change.
4. Run `dimos run m20-dan-nav-sim --help` in another shell.
5. Observe that the new value appears in the help output because this is a
   separate process that re-imports the blueprint.

Verify the actual running value with:

```bash
ps -ef | grep mls_planner
```

The native command line remains unchanged until the DimOS run is restarted.

## Root Cause

The blueprint performs profile loading during Python module import:

```text
m20_dan_nav.py import
  -> yaml.safe_load(mujoco_sim.yaml)
  -> blueprint construction
  -> module deployment
  -> native MLS process startup
```

There is no file watcher, configuration revision signal, or dynamic update RPC
for this profile. `--help` follows the same import path in a new process, which
creates the appearance of hot reload.

## Current Operational Rule

Until this issue is addressed, every persistent YAML edit requires a full
simulation restart:

```bash
dimos stop
dimos --rerun-open none run m20-dan-nav-sim
```

No native rebuild is required for a YAML-only change. The new values should be
verified in the run log or the live `mls_planner` process arguments after the
restart.

## Impact

- Operators may believe they are testing new geometry or sensor values while
  the running simulator still uses old values.
- A viewer can remain connected while the backend continues using stale
  planning parameters, making the UI appear inconsistent with the edited file.
- Future automated tuning must not write YAML and assume the running planner
  has changed.

## Expected Resolution

The implementation should choose and document one of these explicit behaviors:

### Preferred Short-Term Behavior

Keep restart-only semantics, but make them visible:

- log the profile path, modification timestamp or content hash, and resolved
  module values at startup;
- expose the active configuration through `dimos status` or a diagnostic RPC;
- make `--help` clearly state that it inspects a new process and does not reload
  an existing run;
- optionally reject or warn when the profile changes while a run is active.

### Future Hot-Reload Behavior

If live tuning is required later, implement it as an explicit controlled path:

- watch or externally submit a validated configuration revision;
- update Python modules through a versioned RPC/configuration service;
- send supported values to native MLS through a dynamic configuration channel;
- define which fields are safe to update live and which require restart;
- apply updates atomically and report success or rejection to the operator;
- retain the active configuration revision in the run log.

Hot-reloading geometry or planner safety limits must be treated as a safety
feature with validation and observability, not as an implicit side effect of
editing a file.

## Acceptance Criteria

- Documentation explicitly states whether the selected profile is restart-only
  or hot-reloadable.
- The active run exposes the resolved profile revision and key MLS values.
- Editing YAML cannot silently create a mismatch between operator expectations
  and the running native planner.
- If hot reload is implemented, invalid or unsafe updates are rejected without
  partially updating the planner.
- Tests cover startup loading, stale-value behavior, and the selected reload
  semantics.

## Relevant Files

- `dimos/robot/deeprobotics/m20/config/mujoco_sim.yaml`
- `dimos/robot/deeprobotics/m20/nav/m20_dan_nav.py`
- `dimos/navigation/nav_3d/mls_planner/mls_planner_native.py`
- `dimos/core/native_module.py`
- `dimos/robot/cli/dimos.py`
- `dimos/robot/deeprobotics/m20/nav/mujoco_sim.md`
