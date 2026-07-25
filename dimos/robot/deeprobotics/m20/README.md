# DimOS Official DeepRobotics M20 MuJoCo Integration

This is the reusable English entry point for integrating the official
DeepRobotics M20 MuJoCo model into DimOS. It covers installation, startup,
the model-policy contract, verification, and the boundary between simulation
and real-robot behavior.

Languages: English | [Chinese](/dimos/robot/deeprobotics/m20/README.zh-CN.md)

For runtime tuning, recording, replay, and the detailed sensor profile, read
[the M20 MuJoCo runtime guide](/dimos/robot/deeprobotics/m20/nav/mujoco_sim.md)
after completing this document.

## Reading Order

1. **First installation:** [Scope And Model Identity](#scope-and-model-identity), [Quick Start](#quick-start), and [Prerequisites](#prerequisites).
2. **Integration work:** [Included Components](#included-components), [Runtime Architecture](#runtime-architecture), [Model And Policy Contract](#model-and-policy-contract), and [Configuration](#configuration).
3. **Before relying on results:** [Verification Checklist](#verification-checklist), [Control-Fidelity Review](#control-fidelity-review), [Known Motion Limitation](#known-motion-limitation), and [Validation Summary And Roadmap](#validation-summary-and-roadmap).
4. **Operations and maintenance:** [Recording And Replay](#recording-and-replay), [Troubleshooting](#troubleshooting), and [Updating Official Assets](#updating-official-assets).


## Scope And Model Identity

The assets come from the official
[DeepRoboticsLab/sdk_deploy](https://github.com/DeepRoboticsLab/sdk_deploy)
repository at commit `80e3d40084c4ed151ba6f88b0d55cf1d480aa45e`, under the
BSD-3-Clause license.

The public upstream repository calls the robot **M20**, a 16-DOF wheel-legged
quadruped. It does not identify the model as M20 Pro. This integration must
therefore not be used as evidence that a physical robot internally called M20
Pro has identical dimensions, payload, sensors, calibration, or firmware.

The asset inventory and DimOS-specific changes are recorded in
[SOURCE.md](/dimos/robot/deeprobotics/m20/assets/SOURCE.md). Do not replace
the assets with an unverified third-party model or a Unitree policy.

The RGB and synthetic depth cameras are DimOS simulation additions. The
official SDK asset does not specify the physical M20 camera/lidar models,
intrinsics, scan pattern, timing, or calibration, so these streams must not be
treated as hardware-equivalent sensor models.

## Control-Fidelity Review

**Conclusion:** the simulation matches the official M20 Sim-to-Real SDK's
model-coordinate and low-level ONNX-policy contract, but the current DimOS
real-robot navigation chain does not run that ONNX policy. It is suitable for
navigation integration and policy-level simulation testing. It is **not**
evidence that DimOS real-robot trajectories, dynamics, or safety behavior
match the simulation.

| Review item | Result | Evidence |
| --- | --- | --- |
| Kinematics, inertias, joint ranges, and actuator torque ranges | Matches | A CRLF-normalized diff against official `M20.xml` leaves only scene/light removal, mesh-path and collision-visible-group changes, plus cameras, named sensors, and the `home` keyframe |
| ONNX binary and interface | Matches | Both the vendored file and upstream `policy.onnx` have SHA-256 `0ac99f3093d4a984d7587b88d57300cbf7ec2f788401dfa1570d1e4800568f6b`; the interface is `obs [1,57] -> actions [1,16]` |
| Low-level policy adapter | Matches | DimOS and the official `M20PolicyRunner` use the same joint ordering, observation scaling, action scaling, `Kp=[80,80,80,0]`, `Kd=[2,2,2,0.6]`, and leg-position/wheel-velocity hybrid control |
| Policy rate | Matches | The official stack is a 5 ms state machine with decimation 4, hence 20 ms; DimOS is a 1 ms physics step with 20 substeps, hence 20 ms |
| Real encoder coordinates | Correct by design, not measured | The official SDK converts DDS encoder values to model coordinates using per-joint direction and zero offsets before inference; DimOS already runs in MuJoCo model coordinates and must not apply those hardware offsets again |
| DimOS simulation versus DimOS real-robot control chain | **Does not match; uncalibrated** | Simulation sends `cmd_vel` directly to the ONNX policy. `M20Connection` sends normalized axes through the high-level Patrol UDP interface and does not execute the ONNX policy |
| Real command scale and signs | Unverified | The real connection uses `max_linear=1.0` and `max_angular=1.5`; the official RL keyboard path uses `0.7/0.5/0.7`. The real connection code explicitly marks lateral and yaw signs as unverified |
| Motor and ground-contact dynamics | Unverified | MuJoCo retains official rigid-body, friction, and torque-limit data, but not firmware inner loops, motor bandwidth, current/thermal protection, communication delay, tire-ground parameters, or sensor noise |

Do not use this simulation to accept real speed, turning radius, braking distance,
traversability, or safety clearances until the same `cmd_vel` sequence has been
compared against real odometry, IMU, joint state, and video. First validate
signs and scale with low-speed forward, lateral, and in-place turning tests,
then time-align and compare a recorded trajectory. For low-level Sim-to-Real
validation, use the official SDK's authorized `JOINTS_DATA` / `JOINTS_CMD`
chain.

## Known Motion Limitation

This repository deliberately keeps the official M20 ONNX unchanged. A direct
VM MuJoCo check exposed an open behavior limitation: forward tracking is
stable, but a `0.2 m/s` lateral command has little lateral response, maximum
lateral command has backward coupling, and `+0.7` / `-0.7` yaw commands have
materially asymmetric responses. Manual inverse wheel targets remain nearly
mirror-symmetric, while the official ONNX does not produce mirror-symmetric
wheel targets for signed yaw commands.

Treat lateral and yaw tracking as unvalidated. Do not use them for autonomous
navigation performance or safety acceptance. The limitation is exposed here;
the official ONNX, MJCF, and controller mapping are intentionally unchanged
until a comparison with the upstream runner and physical hardware establishes
the correct policy-level remedy.

## Validation Summary And Roadmap

| Area | Result | Notes |
| --- | --- | --- |
| Official source and policy | Pass | BSD-3-Clause source audited; vendored ONNX is byte-identical to upstream and exposes `obs [1,57] -> actions [1,16]` |
| Model and controller contract | Pass | `nq=23`, `nv=22`, `nu=16`; joint tree, inertia, limits, torque ranges, mapping, PD gains, and 20 ms policy cadence checked against the official runner |
| Rendering and synthetic sensors | Pass | Front RGB plus front/rear 48-row synthetic depth, nonempty point cloud, and no robot self-scan with groups `(0, 1)` |
| DimOS integration | Pass | Focused suite: 36 tests; obstacle suite: 15 tests plus explicit MuJoCo check; simple-nav and DAN blueprints completed bounded starts |
| Packaging and docs | Pass | Wheel contains 21 M20 assets; pre-commit, LFS, large-file, and doclinks checks passed |
| Standstill and forward | Pass | Zero command remains upright; `[0.2,0,0]` for 3 s moves `+0.4884 m` forward with `-0.0018 m` lateral drift |
| Lateral and yaw tracking | Open | Small lateral command has little lateral response; maximum lateral couples backward motion; signed yaw is materially asymmetric |
| Physical robot policy | Unverified | Published real deployment loads the same upstream `policy/policy.onnx`; the current robot was unreachable for read-only hash verification |

Current acceptance is limited to model loading, forward-dominant motion,
synthetic sensors, mapping inputs, navigation wiring, and process lifecycle.
Do not accept lateral/yaw tracking, autonomous-navigation performance, or
safety clearance from this simulation.

The next steps are: reproduce the signed-command matrix in the official runner;
collect matched physical M20 joint/IMU/odometry/video data and policy hash; then
request a corrected policy or retrain with balanced signed-yaw/lateral tracking
and left/right symmetry augmentation. Do not change joint signs or gains until
that comparison shows a DimOS-specific mismatch.

## Included Components

| Component | Location | Purpose |
| --- | --- | --- |
| Official MJCF | `dimos/robot/deeprobotics/m20/assets/deeprobotics_m20.xml` | M20 kinematics, inertias, collisions, actuators, cameras, and home keyframe |
| Official meshes | `dimos/robot/deeprobotics/m20/assets/meshes/` | Visual model for the 16-DOF M20 |
| Official ONNX policy | `dimos/robot/deeprobotics/m20/assets/deeprobotics_m20_policy.onnx` | Locomotion policy, `obs [1,57] -> actions [1,16]` |
| DimOS M20 controller | `dimos/simulation/mujoco/policy.py` | Converts navigation velocity commands to official M20 policy observations and hybrid leg/wheel control |
| Model loader | `dimos/simulation/mujoco/model.py` | Loads the assets, uses 1 ms physics steps, and selects the M20 controller |
| Simulation profile | `dimos/robot/deeprobotics/m20/config/mujoco_sim.yaml` | Sensors and M20 simulation planning envelope |

The assets are tracked in Git and packaged into the wheel. The ONNX file is
ordinary Git data, so checkout with `GIT_LFS_SKIP_SMUDGE=1` remains runnable.

## Quick Start

Use a clean checkout. Do not add this integration directly to a working tree
that contains unrelated M20 navigation work; merge or cherry-pick the M20
integration commit instead.

```sh skip
git clone https://github.com/T-Markus-Liang/dimos_m20.git ~/work/dimos_m20
cd ~/work/dimos_m20
git fetch origin codex/m20-official-mujoco-model
git switch --track origin/codex/m20-official-mujoco-model
uv sync --extra all
```

Verify that the model and policy are present before starting a blueprint:

```sh skip
test -s dimos/robot/deeprobotics/m20/assets/deeprobotics_m20.xml
test -s dimos/robot/deeprobotics/m20/assets/deeprobotics_m20_policy.onnx
uv run --no-sync python -c 'from dimos.simulation.mujoco.model import _get_m20_asset_dir; print(_get_m20_asset_dir())'
```

Start the simple-navigation simulation:

```sh skip
cd ~/work/dimos_m20
uv run --no-sync dimos --rerun-open none run m20-simple-nav-sim
```

Start the DAN planner and holonomic-controller simulation:

```sh skip
cd ~/work/dimos_m20
uv run --no-sync dimos --rerun-open none run m20-dan-nav-sim
```

Stop a prior DimOS process before changing blueprints:

```sh skip
cd ~/work/dimos_m20
uv run --no-sync dimos stop
```

## Prerequisites

| Requirement | Reason | Check |
| --- | --- | --- |
| Linux x86_64 or Linux aarch64 | Supported DimOS and MuJoCo runtime platform | `uname -m` |
| Python and uv | Installs the locked DimOS environment | `uv --version` |
| Git | Retrieves project history and branches | `git --version` |
| Git LFS | Other DimOS assets can use LFS, though this ONNX does not | `git lfs version` |
| DimOS native tools | Ray tracing requires them; DAN also needs the MLS planner | `nix --version`, `cargo --version` |
| Headless EGL or a display | MuJoCo RGB and depth rendering | `echo "$DISPLAY"` or `echo "$MUJOCO_GL"` |

The M20 model itself needs neither a ROS 2 process nor a separate
DeepRobotics SDK installation. DimOS directly loads the committed MJCF and
ONNX policy. A missing `nix`, incompatible Rust/Cargo, or missing native
executable is a DimOS environment issue, not an M20 asset issue.

## Runtime Architecture

1. A navigation planner or teleoperation publishes a `cmd_vel` command.
2. `M20MujocoSimConnection` forwards it to the shared-memory MuJoCo process.
3. `M20OnnxController` forms the official policy observation, evaluates the
   ONNX policy, and applies leg-position and wheel-velocity PD torques.
4. MuJoCo RGB/depth cameras publish `color_image` and
   `dimos/slam_aligned_points` for mapping and navigation.

The DimOS adapter adds `head_camera` for front RGB and uses
`lidar_front_camera` plus `lidar_rear_camera` for active synthetic depth. The
legacy left/right depth cameras remain in the MJCF but are not rendered by the
default profile. Visual geometry is in MuJoCo group `2` and collision geometry
in group `3`; the default point-cloud profile renders groups `(0, 1)` so the
robot is not inserted into its own map.

## Model And Policy Contract

| Contract | Value |
| --- | --- |
| Generalized position / velocity dimensions | `nq=23`, `nv=22` |
| Actuators | `nu=16` |
| Robot actuator order | FL, FR, HL, HR; each leg is hip-x, hip-y, knee, wheel |
| Policy observation | 57 values: angular velocity, projected gravity, command, joint state, previous action |
| Policy action | 16 values: 12 leg position targets and 4 wheel velocity targets |
| Physics / policy rates | 1 ms physics step, 20 ms policy update |
| Start pose | M20 standing `home` keyframe at 0.58 m base height |
| Simulation envelope | 0.70 m high, 0.50 m radial clearance |

The real M20 navigation profile deliberately remains different: it uses a
1.00 m overhead envelope and 0.55 m hard-wall clearance for physical hardware.
Do not reduce real-robot safety values to match the simulator.

## Configuration

Edit [mujoco_sim.yaml](/dimos/robot/deeprobotics/m20/config/mujoco_sim.yaml)
for checked-in sensor or simulation-envelope changes, then restart DimOS.
One-off overrides do not require source edits:

```sh skip
cd ~/work/dimos_m20
uv run --no-sync dimos --rerun-open none run m20-simple-nav-sim \
  --option m20mujocosimconnection.pointcloud_fps=1.0 \
  --option m20movingobstacle.enabled=false
```

The normal profile publishes only front 640 x 360 RGB at 10 Hz and a merged
front/rear point cloud at 2 Hz. Each synthetic depth view uses 640 horizontal
samples, 48 vertical rows, and a 10 m range. The 48 rows approximate reduced
vertical lidar channels; they do not reproduce a calibrated physical 48-line
scan pattern or prove equivalence to the real M20 sensors. The mocap person is
visible to cameras but does not collide with M20 by default.

## Verification Checklist

Run this after changing the model, controller, or profile:

```sh skip
cd ~/work/dimos_m20
uv run --no-sync python -m pytest -q \
  dimos/simulation/mujoco/test_m20_policy.py \
  dimos/simulation/mujoco/test_mujoco_process.py \
  dimos/robot/deeprobotics/m20/nav/test_m20_simple_nav_sim.py \
  dimos/robot/deeprobotics/m20/nav/test_m20_dan_nav_sim.py
```

The minimum passing result is a valid model contract, finite controller output,
both blueprints resolving `robot_model="deeprobotics_m20"`, nonempty RGB and
point-cloud streams, and a MuJoCo process that starts and stops cleanly. Run a
bounded headless startup before an interactive test:

```sh skip
cd ~/work/dimos_m20
timeout --signal=INT --kill-after=10s 30s \
  uv run --no-sync dimos --rerun-open none run m20-simple-nav-sim
```

## Recording And Replay

Start Rerun before DimOS to record an `.rrd`; the recorder must own port
`9877` before the bridge starts. SQLite `nav-record` limitations and replay
commands are documented in
[the M20 MuJoCo runtime guide](/dimos/robot/deeprobotics/m20/nav/mujoco_sim.md#recording-and-replay).

## Troubleshooting

| Symptom | Likely cause | Action |
| --- | --- | --- |
| `Unknown robot policy: deeprobotics_m20` | Checkout predates the integration | Fetch the branch or merge commit `014403f5` |
| `Error opening file '*.STL'` | Assets are absent from checkout or wheel | Verify `assets/meshes/`, then run `uv sync` |
| ONNX session cannot load | Policy is absent, corrupt, or replaced | Restore the tracked `deeprobotics_m20_policy.onnx` and verify its 57/16 interface |
| `nix: not found` | DimOS native environment is incomplete | Install/configure Nix or use a native-built checkout |
| Cargo cannot parse `Cargo.lock` | Rust/Cargo is too old | Update Rust/Cargo and rebuild the MLS executable |
| Robot appears in its own point cloud | Depth groups include `2` or `3` | Keep `pointcloud_geom_groups: [0, 1]` |
| M20 does not move | Spawn is blocked or command does not reach `cmd_vel` | Test from `(-1, 1)`, inspect `dimos/slam_odom`, then inspect `cmd_vel` |
| A colleague calls it M20 Pro | Public source identifies only M20 | Confirm mechanical and sensor equivalence with the hardware owner |

## Updating Official Assets

1. Record the upstream repository URL, immutable commit, license, and source
   paths in `assets/SOURCE.md`.
2. Compare MJCF names, joint and actuator order, camera names, dimensions, and
   ONNX input/output signatures before replacing any asset.
3. Reuse the M20 controller mapping only when the policy contract is identical;
   otherwise implement and test a new controller contract.
4. Run the verification checklist, a bounded startup for both blueprints, and
   a visual RGB/depth check.
5. Update both READMEs, `nav/mujoco_sim.md`, and the source attribution in the
   same commit.

Never substitute a visual mesh for a matched locomotion policy and actuator
contract. A robot that merely looks correct is not a usable navigation
simulation.
