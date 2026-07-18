# Issue: Separate M20 Real and Simulation Configuration Profiles

- Status: Open
- Priority: High
- Scope: M20 Dan navigation startup and configuration architecture
- Affected blueprints: `m20-dan-nav`, `m20-dan-nav-sim`
- Discovered on: 2026-07-14

## Problem

The real M20 and MuJoCo startup commands use different connection adapters, but
they share the same navigation core and all of its module parameters.

```text
m20-dan-nav
  -> M20Connection
  -> shared _m20_dan_nav_core

m20-dan-nav-sim
  -> M20MujocoSimConnection
  -> shared _m20_dan_nav_core
```

The shared core currently gives both runs identical mapping, MLS planning,
local planning, tracking-control, movement-management, and visualization
settings. This is incorrect while the simulator uses a Unitree Go1 model
instead of an M20 model.

The configuration system also lacks a clear platform-profile layer. Robot
geometry and capability values are duplicated as Python constants and then
passed into individual module blueprints. It is difficult to determine the
final configuration of a running system or the source of each value.

## Evidence

The two assembled blueprints currently have no parameter differences among
their 11 common modules.

| Area | Real M20 | Current MuJoCo simulation |
| --- | --- | --- |
| Connection | `M20Connection` | `M20MujocoSimConnection` |
| Global `robot_model` | `m20` | `unitree_go2` |
| Global `simulation` | empty | `mujoco` |
| Mapping parameters | shared | shared |
| MLS parameters | shared | shared |
| DanLocalPlanner parameters | shared | shared |
| DanHolonomicTC parameters | shared | shared |

The simulation label is currently misleading. The simulation blueprint sets
`GlobalConfig.robot_model="unitree_go2"`, but `mujoco_process.py` explicitly
maps `unitree_go2` to `unitree_go1`. The running simulator therefore loads:

- `data/mujoco_sim/unitree_go1.xml`;
- `data/mujoco_sim/unitree_go1_policy.onnx`;
- `Go1OnnxController`.

The repository contains ONNX locomotion policies only for Go1 and G1. The
installed MuJoCo Menagerie package contains a Go2 MJCF model, but no compatible
Go2 locomotion checkpoint or DimOS `Go2OnnxController`. A true Go2 simulation
profile therefore requires policy integration, observation/action and joint
ordering validation, and simulation sensor/camera attachment. Reusing or
renaming the Go1 policy would not constitute a consistent Go2 simulation.

Current MLS values are defined in
`dimos/robot/deeprobotics/m20/nav/m20_dan_nav.py` and include:

```text
robot_height     = 1.00 m
wall_clearance_m = 0.55 m
step_threshold_m = 0.15 m
```

The active simulator can physically enter spaces that this larger planning
envelope rejects. After entering beneath or beside the table in the current
MuJoCo scene, MLS can report that the start and goal are on separate connected
surface components. An offline sensitivity check restored paths with a smaller
simulation envelope, confirming that the mismatch is material.

Additional configuration inconsistencies:

- `m20_width_clearance=0.45` is used both as a full robot width and as the base
  of a radial hard wall clearance. These quantities have different semantics.
- M20 geometry constants are duplicated in multiple navigation blueprints.
- A source comment describes the head as approximately 1.3 m high, while the
  configured planning height is 1.0 m.
- `GlobalConfig.robot_width` and `robot_rotation_diameter` exist, but the active
  M20 MLS chain does not consume them.
- `dimos/robot/deeprobotics/m20/config/mujoco_sim.yaml` configures the
  simulation sensor adapter and temporary Go1 MLS envelope. It is not yet a
  complete simulation run profile.
- `dimos show-config` displays `GlobalConfig`, not the fully resolved config of
  every module in the selected blueprint.

## Current Parameter Resolution

Module values are resolved in this order, from lowest to highest precedence:

```text
Pydantic module defaults
  < blueprint keyword arguments
  < --config file values
  < environment overrides
  < --option / -o command-line overrides
```

`Blueprint.global_config(...)` provides runtime-wide defaults. The coordinator
then injects the resolved `GlobalConfig` into every module as `config.g`.
Native modules serialize their resolved module config for their Rust process.

The name `GlobalConfig` must not imply that all shared robot-domain parameters
belong there. It is primarily the runtime/infrastructure configuration surface.

## Temporary Mitigation

The first implementation step keeps the existing Go1 MJCF and Go1 ONNX policy
and gives `m20-dan-nav-sim` a Go1-specific MLS envelope through
`config/mujoco_sim.yaml`:

```text
robot_height     = 0.50 m
wall_clearance_m = 0.45 m
```

The nominal Go1 MJCF standing pose measures approximately 0.373 m high with a
maximum horizontal radius of approximately 0.399 m. The selected planning
values include about 0.127 m of vertical margin and 0.051 m of lateral margin.
The real `m20-dan-nav` entrypoint retains its existing M20 values (`1.00 m` and
`0.55 m`). This resolves the immediate model/envelope mismatch but does not
close the broader configuration-architecture issue.

## Expected Configuration Boundaries

Use three explicit layers:

### 1. Runtime and Infrastructure Configuration

Keep process-wide settings in `GlobalConfig`, for example:

- transport backend and Zenoh networking;
- worker count;
- robot address and device paths;
- simulation/replay mode;
- viewer and listening endpoints;
- selected robot model identifier.

### 2. Platform Profile

Add a validated source of truth for cross-module robot properties:

- body width, length, and height;
- planning or circumscribed radius;
- rotation envelope;
- maximum traversable step;
- lateral and overhead safety margins;
- velocity, acceleration, and angular-rate limits;
- sensor extrinsics and platform capabilities.

At minimum, define separate profiles for:

- the real DeepRobotics M20;
- the current Unitree Go1 MuJoCo model used by the M20 adapter.

For the immediate consistency fix, align the simulation planner envelope with
the actual Go1 MJCF and Go1 policy. Treat migration to a real Go2 MJCF plus a
matching Go2 locomotion policy as a separate follow-up unless that policy is
first supplied and validated.

Physical dimensions and safety margins must be represented as separate fields.
Do not lower real-M20 safety values merely to make the smaller simulator model
pass through an environment.

### 3. Run and Module Configuration

Keep algorithm-specific values in validated module configs:

- voxel-map integration and publication settings;
- MLS graph and cost parameters;
- local path gating and smoothing;
- tracking-controller gains and tolerances;
- visualization rates and memory limits;
- simulator sensor rendering settings.

A small profile-driven blueprint builder should resolve platform and run
profiles into final per-module configs while keeping explicit real and
simulation connection adapters.

## Required Behavior

The resulting startup structure should be equivalent to:

```python
m20_dan_nav = build_dan_nav(
    platform=M20_REAL_PROFILE,
    runtime=M20_REAL_NAV_PROFILE,
    connection=M20Connection.blueprint(),
)

m20_dan_nav_sim = build_dan_nav(
    platform=GO1_MUJOCO_PROFILE,
    runtime=GO1_MUJOCO_NAV_PROFILE,
    connection=M20MujocoSimConnection.blueprint(),
)
```

This example describes the ownership boundary, not a required API shape.

## Acceptance Criteria

- Real and simulation entrypoints continue to use mutually exclusive connection
  adapters; the simulation blueprint cannot command real hardware.
- Real M20 and current MuJoCo geometry/capability profiles are separate,
  validated, and named explicitly.
- Shared navigation code remains reusable, but profile-derived MLS and control
  values may intentionally differ between real and simulation blueprints.
- Robot width is not reused as a radial wall clearance without an explicit,
  documented conversion.
- Active duplicate geometry literals are removed from M20 navigation blueprints.
- Existing `--config` and `--option module.field=value` overrides continue to
  work and retain the documented precedence.
- Startup logs include the selected platform/run profile and the final resolved
  configuration for each module, including native-module values.
- Tests assert connection isolation and the intentional real/simulation
  parameter differences.
- Documentation explains which parameters are global runtime settings,
  cross-module platform properties, and module-local algorithm settings.

## Verification

- Run blueprint-assembly tests without starting hardware.
- Inspect both resolved blueprint configs and confirm expected differences.
- Start `m20-dan-nav-sim` and confirm the logged MLS process values match the
  simulation profile.
- Confirm valid floor goals remain plannable near and after interaction with the
  table scene, within the intended simulated robot envelope.
- Before real deployment, verify M20 dimensions and motion limits from an
  authoritative specification or direct measurement.

## Non-Goals

- Replacing the MLS algorithm.
- Claiming that the current Go1 simulation validates M20 dynamics, gait,
  actuator limits, or physical performance.
- Migrating the simulator to a true Go2 model and locomotion policy before a
  compatible, traceable policy artifact is available.
- Selecting final real-M20 dimensions without authoritative measurements.
- Implementing orientation-aware non-circular footprint planning in this issue.

## Relevant Files

- `dimos/robot/all_blueprints.py`
- `dimos/robot/cli/dimos.py`
- `dimos/core/global_config.py`
- `dimos/core/coordination/blueprints.py`
- `dimos/core/coordination/module_coordinator.py`
- `dimos/robot/deeprobotics/m20/nav/m20_dan_nav.py`
- `dimos/robot/deeprobotics/m20/nav/m20_simple_nav.py`
- `dimos/robot/deeprobotics/m20/config/mujoco_sim.yaml`
- `dimos/navigation/nav_3d/mls_planner/mls_planner_native.py`
- `dimos/navigation/nav_3d/mls_planner/rust/src/mls_planner.rs`
