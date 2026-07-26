# DeepRobotics M20 MuJoCo Integration

This directory contains the DimOS integration for the DeepRobotics M20,
including a MuJoCo simulation, an official locomotion policy, real-robot
transport adapters, and navigation blueprints.

> [!IMPORTANT]
> The M20 support and the validation results documented here are experimental.
> The simulation is suitable for navigation integration and regression testing,
> but it is not a substitute for physical-robot calibration or safety testing.

Languages: English | [Chinese](/dimos/robot/deeprobotics/m20/README.zh-CN.md)

## Documentation Map

| Document | Scope |
| --- | --- |
| This README | Installation, architecture, configuration ownership, validation evidence, and known limitations |
| [MuJoCo runtime reference](/dimos/robot/deeprobotics/m20/nav/mujoco_sim.md) | Blueprint commands, Viewer operation, YAML parameter reference, recording, and troubleshooting |
| [Asset source record](/dimos/robot/deeprobotics/m20/assets/SOURCE.md) | Upstream repository, immutable source commit, license, and DimOS-specific MJCF changes |

## Scope And Provenance

The model, meshes, and locomotion policy come from the official
[DeepRoboticsLab/sdk_deploy](https://github.com/DeepRoboticsLab/sdk_deploy)
repository at commit `80e3d40084c4ed151ba6f88b0d55cf1d480aa45e` under the
BSD-3-Clause license.

The public source calls this 16-DOF wheel-legged quadruped **M20**. It does not
use the name M20 Pro. This integration therefore does not establish that a
physical product called M20 Pro has identical geometry, sensors, calibration,
firmware, or payload.

The vendored ONNX SHA-256 is
`0ac99f3093d4a984d7587b88d57300cbf7ec2f788401dfa1570d1e4800568f6b`,
which matches the policy in the pinned upstream source. The upstream deployment
instructions also load that policy path on the robot. A deployed robot may
still contain a vendor or site-specific replacement; verify its hash before
claiming binary identity.

DimOS keeps the official robot geometry, inertial properties, joint limits,
actuators, and policy. It removes the source scene floor and light, assigns
geometry groups for synthetic sensing, adds named cameras and sensors, and
adds a standing keyframe. See [the source record](/dimos/robot/deeprobotics/m20/assets/SOURCE.md) for the
complete boundary.

The RGB camera and RoboSense Airy lidar simulation are DimOS additions, not
vendor-calibrated M20 sensor models. The real Airy specification is 360 x 90
degrees with 96-channel operation and a decoder range up to 60 m. The default
simulation uses two outward-facing 180 x 90-degree sectors, 96 vertical lines,
64 azimuth samples per unit, 2 Hz publication, and a 10 m cutoff to keep the
CPU-only test environment responsive. Exact factory beam angles and M20 mount
transforms are not available in the public sources.

## Included Components

| Component | Location | Responsibility |
| --- | --- | --- |
| Official MJCF and meshes | `assets/` | M20 kinematics, dynamics, collisions, actuators, and visual geometry |
| Official ONNX policy | `assets/deeprobotics_m20_policy.onnx` | `57`-value observation to `16`-value locomotion action |
| MuJoCo policy adapter | `dimos/simulation/mujoco/policy.py` | Policy observation, joint mapping, and hybrid leg-position/wheel-velocity control |
| Simulation connection | `mujoco_sim.py` | M20 topic publication and simulation-only yaw command adaptation |
| Sensor and planner profile | `config/mujoco_sim.yaml` | Checked-in runtime values used by both simulation blueprints |
| Rerun layout | `blueprints/basic.py` | Front-camera and 3D visualization layout and display-only rate controls |
| Navigation blueprints | `nav/m20_simple_nav.py`, `nav/m20_dan_nav.py` | Real and simulated Simple A* and DAN compositions |

The retained `nav/` files have distinct responsibilities:

| Path | Responsibility |
| --- | --- |
| `m20_simple_nav.py` | Real and simulated Simple Nav blueprints |
| `m20_dan_nav.py` | Real and simulated DAN navigation blueprints |
| `moving_obstacle.py` | Reproducible mocap-person stimulus used by Simple Nav simulation |
| `odom2posestamped.py` | Odometry-to-pose adapter required by the DAN stack |
| `m20_map_save.py` | Complete physical-M20 map-recording blueprint |
| `map_save/` | Native point-cloud accumulation module used by `m20-map-save` |
| `test_m20_*.py` | Current blueprint, sensor-profile, and moving-obstacle regression tests |
| `mujoco_sim.md` | Runtime and parameter reference |

## Supported Blueprints

This table covers blueprints owned by `dimos.robot.deeprobotics.m20`.
Similarly named entries from `dimos.robot.m20` and `dimos.robot.m20_native`
are separate integrations outside this document's scope.

| Blueprint | Runtime target | Purpose |
| --- | --- | --- |
| `m20` | Physical M20, command capable | Base connection, TF, front/rear image visualization, and Rerun |
| `m20-simple-nav` | Physical M20, command capable | Physical-robot Simple A* navigation |
| `m20-dan-nav` | Physical M20, command capable | Physical-robot MLS and DAN navigation |
| `m20-map-save` | Physical M20, command capable | Accumulate registered SLAM point clouds and save a PCD on graceful shutdown |
| `m20-dds-rerun` | Physical M20 onboard, sensors only | Bridge onboard lidar, IMU, and odometry from drdds to LCM and Rerun |
| `m20-simple-nav-sim` | MuJoCo | Official-M20 simulation with Simple Nav and a moving person |
| `m20-dan-nav-sim` | MuJoCo | Official-M20 simulation with MLS and DAN control |

> [!WARNING]
> The four command-capable physical-robot blueprints instantiate
> `M20Connection` and can send commands to hardware. Use the robot's normal
> network, operator, remote emergency stop, and clear-area procedures. Do not
> use simulation limits as physical safety limits.

The map-save blueprint writes
`nav/map_save/m20_accumulated_map.pcd` by default. It is a dedicated physical
mapping workflow, not a second navigation implementation.

## Architecture

1. Teleoperation or a navigation controller publishes `cmd_vel`.
2. `M20MujocoSimConnection` forwards the command to the MuJoCo subprocess.
3. `M20OnnxController` builds the official 57-value observation, evaluates the
   ONNX policy every 20 ms, and applies 12 leg-position and four wheel-velocity
   targets.
4. MuJoCo publishes odometry, front RGB, and a merged front/rear synthetic
   point cloud.
5. Mapping, planning, control, TF, and Rerun modules consume those streams in
   the same DimOS composition.

| Blueprint | Planning and tracking chain | Moving person |
| --- | --- | --- |
| `m20-simple-nav-sim` | `CostMapper -> ReplanningAStarPlanner -> LocalPlanner/PController` | Enabled by default |
| `m20-dan-nav-sim` | `MLSPlannerNative -> DanLocalPlanner -> DanHolonomicTC` | Not included |

Both simulation blueprints force the MuJoCo window off. `--rerun-open` controls
only the Rerun Viewer. Neither simulation blueprint instantiates
`M20Connection`, so it cannot send commands to a physical robot.

## Installation

Follow the repository's [Ubuntu installation guide](/docs/installation/ubuntu.md)
for system packages, Nix, and native build prerequisites. A source checkout
with the CPU ONNX and simulation extras can then be prepared with:

```sh skip
git clone https://github.com/dimensionalOS/dimos.git
cd dimos
uv sync --extra cpu --extra sim
source "$HOME/.cargo/env"
```

During review of an unmerged branch, replace the clone URL and switch command
with the contributor fork and PR branch under test. Do not hard-code a personal
fork or a temporary integration branch into deployment automation.

Confirm that the committed assets and Python environment are usable:

```sh skip
test -s dimos/robot/deeprobotics/m20/assets/deeprobotics_m20.xml
test -s dimos/robot/deeprobotics/m20/assets/deeprobotics_m20_policy.onnx
uv run --no-sync python -c \
  'from dimos.simulation.mujoco.model import _get_m20_asset_dir; print(_get_m20_asset_dir())'
```

The MuJoCo model does not require ROS 2 or a separate DeepRobotics SDK. The
navigation stack does require the normal DimOS native mapping/planning tools.
The onboard `m20-dds-rerun` bridge is a separate physical deployment and does
require the robot's drdds SDK plus its native CMake toolchain.

## Run The Simulation

Stop an earlier coordinator before starting another blueprint:

```sh skip
uv run --no-sync dimos stop
```

Run Simple Nav with headless EGL MuJoCo and a native Rerun Viewer:

```sh skip
MUJOCO_GL=egl uv run --no-sync dimos --rerun-open native run m20-simple-nav-sim
```

For a server or CI session without a desktop:

```sh skip
MUJOCO_GL=egl uv run --no-sync dimos --rerun-open none run m20-simple-nav-sim
```

Run the DAN stack by replacing the blueprint name:

```sh skip
MUJOCO_GL=egl uv run --no-sync dimos --rerun-open native run m20-dan-nav-sim
```

If an SSH-launched process cannot inherit the desktop session, start DimOS with
`--rerun-open none`, then launch the Viewer from the desktop environment:

```sh skip
uv run --no-sync dimos-viewer --connect rerun+http://127.0.0.1:9877/proxy
```

See the [runtime reference](/dimos/robot/deeprobotics/m20/nav/mujoco_sim.md) for goal testing, recording, and
process cleanup.

## Configuration Ownership

The checked-in runtime profile is
[`config/mujoco_sim.yaml`](/dimos/robot/deeprobotics/m20/config/mujoco_sim.yaml). Edit it for persistent
changes and restart DimOS. The YAML is validated when a simulation blueprint is
imported; unknown keys and invalid ranges fail startup instead of being ignored.

| YAML section | Consumed by | Configures |
| --- | --- | --- |
| `m20mujocosimconnection` | Both simulation blueprints | RGB, synthetic lidar, person collision flag, and simulation yaw adaptation |
| `m20movingobstacle` | Simple Nav only | Simulated person's route, speed, update rate, and robot-proximity behavior |
| `mlsplannernative` | Both simulation blueprints | Simulation robot height and radial wall clearance |
| `replanningastarplanner` | Simple Nav only | Raw-path diagnostics, constrained smoothing, backtracking, and shadow validators |

One-off overrides use the generated module name as the option prefix:

```sh skip
uv run --no-sync dimos --rerun-open none run m20-simple-nav-sim \
  --option m20mujocosimconnection.pointcloud_fps=1.0 \
  --option m20movingobstacle.enabled=false
```

YAML supplies module parameters; it does not instantiate modules. For example,
`M20MovingObstacle` must remain part of `m20-simple-nav-sim` for its YAML section
to have any effect. It drives a mocap person and is not a robot emergency-stop
or obstacle-avoidance safety module.

Not every simulation property is YAML-controlled:

| Property | Source of truth | Status |
| --- | --- | --- |
| Sensor mount positions and orientations | `assets/deeprobotics_m20.xml` | DimOS approximation; not measured M20 extrinsics |
| Camera optical TF and `CameraInfo` | `tf.py` | TODO calibration; current static 1280 x 720 intrinsics are not synchronized with the 640 x 360 simulation stream |
| Ray-generation algorithm | `dimos/simulation/mujoco/mujoco_process.py` | Shared implementation; change with tests |
| Rerun layout and display-only downsampling | `blueprints/basic.py` | Simulation-specific front-camera layout |
| Policy cadence, gains, and joint mapping | `dimos/simulation/mujoco/policy.py` | Official policy contract; not a runtime tuning surface |
| Robot geometry and dynamics | MJCF and ONNX assets | Do not edit without upstream or physical evidence |

The complete current YAML values and constraints are listed in the
[runtime parameter reference](/dimos/robot/deeprobotics/m20/nav/mujoco_sim.md#parameter-reference).

## Verification Procedure

### Automated Tests

Stop all DimOS, MuJoCo, Rerun, mapping, and planner processes before testing.
Run the current M20 and shared MuJoCo regression set:

```sh skip
MUJOCO_GL=egl uv run --no-sync python -m pytest -q \
  dimos/robot/deeprobotics/m20 \
  dimos/simulation/mujoco/test_m20_policy.py \
  dimos/simulation/mujoco/test_mujoco_process.py \
  dimos/robot/unitree/test_mujoco_connection.py
```

The moving-person integration test is marked separately because it creates a
MuJoCo scene:

```sh skip
MUJOCO_GL=egl uv run --no-sync python -m pytest -q -m mujoco \
  dimos/robot/deeprobotics/m20
```

Run a bounded full-process startup before interactive goal testing:

```sh skip
timeout --signal=INT --kill-after=10s 30s \
  env MUJOCO_GL=egl uv run --no-sync dimos --rerun-open none \
  run m20-simple-nav-sim
```

### Reference Test Environment

The latest documented validation was performed on 2026-07-26 at revision
`ee818561d9999df73b7ac8baad939a44229dff57` with these conditions:

| Item | Value |
| --- | --- |
| Host | Ubuntu 22.04.5 LTS aarch64 VM, 14 Apple virtual CPUs, 62 GiB RAM |
| MuJoCo / Rerun SDK | 3.5.0 / 0.32.0-alpha.1 |
| MuJoCo rendering | Headless EGL |
| Viewer | Native Rerun on the VM desktop, OpenGL `llvmpipe` software rendering |
| Scene/profile | Default office scene and committed `mujoco_sim.yaml`; front RGB, two lidar sectors, moving person enabled |
| Process discipline | Existing DimOS/MuJoCo/Rerun processes stopped before every run |

### Results Recorded On The Reference Revision

| Test | Input and conditions | Result |
| --- | --- | --- |
| Automated M20/shared MuJoCo suite | Command above, default marker selection | `81 passed, 1 deselected` in 7.95 s |
| Explicit MuJoCo obstacle scene | `-m mujoco dimos/robot/deeprobotics/m20` | `1 passed, 53 deselected` in 7.05 s |
| Simple Nav smoke test | 30 s `m20-simple-nav-sim`, EGL, no Viewer | All 11 modules started; ONNX loaded; RGB 7.70-7.91 FPS; lidar 1.98-1.99 Hz; moving-person pause, redirect, and resume executed; shutdown left no process or listener |
| DAN smoke test | 30 s `m20-dan-nav-sim`, EGL, no Viewer | All 12 modules, voxel mapper, MLS planner, DAN controller, ONNX, and MuJoCo started; shutdown left no process or listener |
| Model contract | Direct MJCF load | `nq=23`, `nv=22`, `nu=16`; expected joint/actuator and `obs [1,57] -> actions [1,16]` policy contracts passed |
| Standstill | 2 s zero command | Base settled from 0.58 m to 0.5636 m; finite controls; remained upright |
| Forward motion | `[0.2, 0, 0]` for 3 s after settling | `+0.4884 m` forward, `-0.0018 m` lateral; remained upright |
| Current sensor timing | 640 x 360 RGB at 8 FPS; two 64 x 96 sectors at 2 Hz; Viewer open | RGB about 7.73 FPS, P95 frame interval 133.5 ms, maximum 135.6 ms; merged cloud about 1.8-2.0 Hz |
| Rerun layout | Simulation front RGB only; image entities excluded from 3D | No unused rear-camera loading view and no red `3D -> color_image` mismatch; Viewer CPU observed around 2-3% after settling |
| Navigation goal | Goal `(-3.35, -0.51)` from approximately `(-2.457, 0.977)` | Planner entered `path_following`, robot moved to approximately `(-3.270, -0.258)`, and `goal_reached` was published |
| Normal teleop yaw | `angular.z=+0.8` and `-0.8`, each for 3 s; scale 2.0, cap 1.6 | Approximately `+0.585 rad` and `-1.112 rad`; both directions turn, but response remains asymmetric |

These are scenario results, not statistical performance guarantees. The direct
motion measurements were collected when the official asset integration was
introduced; the official MJCF, ONNX, policy adapter, and 20 ms policy cadence
have remained unchanged since that run.

## Known Limitations

1. **No physical equivalence claim.** The DimOS real-robot path uses the Patrol
   UDP high-level interface, while simulation sends velocity commands to the
   official low-level ONNX policy. Real speed, turning radius, braking,
   traversability, and safety clearance remain unvalidated.
2. **Lateral and signed-yaw fidelity is open.** A controlled 3 s test produced
   only `+0.0270 m` lateral motion for `[0,0.2,0]`; `[0,0.5,0]` moved laterally
   but coupled `-0.2337 m` backward. `+0.7` and `-0.7` yaw produced `+0.2200`
   and `-0.7311 rad`. Tracing showed asymmetric wheel targets from the official
   ONNX, while manually mirrored wheel targets produced nearly mirrored yaw.
   The policy is intentionally unchanged.
3. **Sensor calibration is approximate.** Public sources do not provide Airy
   DIFOP beam tables, M20 lidar/camera extrinsics, timing, noise, motion
   distortion, or blind-zone models. The simulation profile intentionally
   reduces azimuth density, range, and update rate.
4. **Camera calibration needs consolidation.** `tf.py` still publishes TODO
   1280 x 720 intrinsics while the simulation image is 640 x 360. Do not use the
   current image/3D overlay for calibrated projection measurements.
5. **Software rendering can still stall briefly.** On `llvmpipe`, heavy global
   map updates or goal transitions can produce isolated long frames even though
   the regular loading/stall condition has been removed.
6. **The moving person is a perception stimulus.** Its collision is disabled
   because a prescribed mocap body can unrealistically push over the robot. Its
   proximity logic redirects the person; it does not stop the M20.
7. **Broader repository checks have external dependencies.** One inherited
   `office_lidar` test needs private LFS credentials, and unrelated Go2/FastLIO
   blueprint-kwargs checks may fail before reaching M20 code. Record these as
   test-environment exclusions rather than M20 passes.
8. **Bounded shutdown reports shared-memory cleanup.** The 30 s SIGINT smoke
   test exits with a Python `resource_tracker` warning for seven shared-memory
   objects. No child process or listener remains afterward, but graceful
   shared-memory lifecycle cleanup still needs investigation.

Do not tune joint signs, gains, MJCF dynamics, or the ONNX policy merely to hide
these limitations. Reproduce the command matrix in the official runner and
collect synchronized physical M20 odometry, IMU, joint state, and video before
changing the model-policy contract.

## Recording And Replay

An external Rerun recorder must bind port `9877` before DimOS starts if the run
must be saved as `.rrd`. A `nav-record` SQLite recorder must be composed at
blueprint startup and records only explicitly connected streams. Commands,
stream coverage, and replay limitations are documented in
[the runtime reference](/dimos/robot/deeprobotics/m20/nav/mujoco_sim.md#recording-and-replay).

## Troubleshooting

| Symptom | Cause or check |
| --- | --- |
| `Unknown robot policy: deeprobotics_m20` | The checkout predates the M20 integration or is missing the simulation changes |
| `Error opening file '*.STL'` | Verify `assets/meshes/` and reinstall/sync the environment |
| ONNX load failure | Restore the tracked policy and verify its SHA-256 and 57/16 interface |
| No RGB in headless mode | Set `MUJOCO_GL=egl`; verify EGL is available |
| Native Viewer exits with `winit` | The process lacks a desktop display; use `--rerun-open none` or launch the Viewer inside the desktop session |
| Red image under the 3D view | Use the simulation-specific Rerun blueprint; image entities belong in `Spatial2DView` |
| Robot appears in its point cloud | Keep `pointcloud_geom_groups: [0, 1]` so robot visual/collision groups are excluded |
| A second run is slow or fails to bind ports | Run `dimos stop`, then verify stale MuJoCo, Rerun, voxel, and MLS processes are gone |

## Updating Official Assets

1. Pin the upstream repository and immutable commit in `assets/SOURCE.md`.
2. Recheck license compatibility and retain the upstream license file.
3. Compare MJCF names, dimensions, joints, actuators, inertias, limits, and
   ONNX signatures before replacing any file.
4. Reuse the controller only if joint ordering and policy contracts are
   identical; otherwise implement a separately reviewed adapter.
5. Run both automated commands, bounded startup for both simulation blueprints,
   sensor inspection, teleop direction checks, and at least one navigation goal.
6. Update this README, the Chinese README, the runtime reference, and source
   attribution in the same change.

A visually correct mesh without its matched actuator and locomotion-policy
contract is not a valid robot simulation.
