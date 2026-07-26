# M20 Navigation MuJoCo Test

For installation, official asset provenance, controller contracts, and the
full integration workflow, read [the English M20 integration README](/dimos/robot/deeprobotics/m20/README.md).
The [Chinese README](/dimos/robot/deeprobotics/m20/README.zh-CN.md) covers the same workflow.

The official DeepRobotics M20 MuJoCo model and locomotion policy can exercise
two M20 navigation chains without opening a MuJoCo window:

| Blueprint | Command | Planning and tracking chain |
| --- | --- | --- |
| Dan navigation | `m20-dan-nav-sim` | MLSPlannerNative -> DanLocalPlanner -> DanHolonomicTC |
| Simple navigation | `m20-simple-nav-sim` | CostMapper -> ReplanningAStarPlanner -> LocalPlanner/PController |

## Simple Navigation Trajectory Test

Use this blueprint when testing the existing A* path generation, 0.1 m path
smoothing/resampling, obstacle-triggered replanning, and the rotate-then-drive
LocalPlanner. It deliberately does not include DanLocalPlanner or
DanHolonomicTC.

```bash
cd /home/markus/work/dimos_m20
source "$HOME/.cargo/env"
source .venv/bin/activate
dimos stop
dimos --rerun-open none run m20-simple-nav-sim
```

The simulator publishes `dimos/slam_odom` and
`dimos/slam_aligned_points`, while `MovementManager` routes planner
`nav_cmd_vel` to MuJoCo through `cmd_vel`. The blueprint uses the checked-in
M20 model envelope from `mujoco_sim.yaml`: 0.70 m height and 0.50 m radial
clearance, resulting in a 1.00 m A* robot width and rotation diameter. This
validates the published M20 MJCF and ONNX control contract, but it does not
replace real-hardware validation of payloads, sensors, traction, or safety limits.

## Recording And Replay

Use an external Rerun server to persist visual diagnostics, including enabled
RGB, SLAM point clouds, maps, TF, and planner visuals. Start it before DimOS:

```bash
mkdir -p /public/M20_dimos
uv run --no-sync rerun --serve-grpc --port 9877 \
  --server-memory-limit 15GB \
  --save "/public/M20_dimos/m20_$(date +%Y%m%d_%H%M%S).rrd"
```

Then start `m20-simple-nav-sim` in a separate terminal with
`--rerun-open none`. The M20 bridge detects the existing port `9877` and
connects to it. Stop DimOS first and then stop the Rerun server to finalize the
RRD. Verify and replay the result with:

```bash
uv run --no-sync rerun rrd verify /public/M20_dimos/run.rrd
uv run --no-sync rerun /public/M20_dimos/run.rrd --memory-limit 8GB
```

The Rerun server must start before DimOS. If DimOS starts first, its bridge
claims port `9877` without `--save`; a later recording server cannot bind that
port or save the earlier events. Restart both processes in the documented
order when that happens.

For a structured SQLite recording, compose the optional recorder:

```bash
mkdir -p /public/M20_dimos/db
uv run --no-sync dimos --rerun-open none run m20-simple-nav-sim nav-record \
  --option "navrecord.db_path=/public/M20_dimos/db/m20-sim.db"
```

`nav-record` records only connected streams. The current M20 Sim composition
records TF and `global_map`; it does not automatically capture
`dimos/slam_odom`, `dimos/slam_aligned_points`, or RGB. Render a DB to RRD with
`dimos mem rerun /path/run.db --out /path/run-from-db.rrd --no-gui`.
See [Navigation Recording And Replay](/docs/usage/navigation_recording_replay.md)
for stream inspection, web replay, and programmatic SQLite replay.

The current DB recorder is part of the initial `dimos run` composition and
cannot be dynamically added to an already-running navigation coordinator. A
future standalone `m20-sim-nav-record` process should subscribe to the same
LCM topics with explicit M20 sensor mappings. That design would permit
mid-run capture of future messages without port `9877` dependency, but it is
not yet an available blueprint.

The simulation also enables one person-shaped moving obstacle. It reuses the
existing MuJoCo mocap person and `/person_pose` transport, so the obstacle is
visible to RGB and synthetic point clouds. Physical contact is disabled by
default because a prescribed mocap body has effectively infinite mass and can
push over the M20 instead of testing perception and replanning. A fixed random
seed chooses between adjacent edges of an office path already used by the
MuJoCo person-follow tests. Disable the obstacle with
`--option m20movingobstacle.enabled=false` when comparing against a static map.
The person also observes `dimos/slam_odom`: when it approaches within 0.9 m of
the robot, it stops for 1 second, reverses along a validated waypoint edge, and
returns to normal random walking after reaching 1.1 m separation. These values
are configured under `m20movingobstacle` in `mujoco_sim.yaml`.

`m20-dan-nav-sim` runs the WD M20 Dan navigation stack against the existing
DimOS MuJoCo simulator without opening a MuJoCo window.

```bash
cd /home/markus/work/dimos_m20
source "$HOME/.cargo/env"
source .venv/bin/activate
dimos stop
dimos --rerun-open none run m20-dan-nav-sim
```

The simulator publishes M20-compatible `slam_odom` and `slam_aligned_points`
topics and consumes `cmd_vel`. The front RGB stream is enabled for simulation
inspection, while the rear image topic remains disabled because the legacy
simulator has no rear camera and would only duplicate the front frame. The
simulator uses the official DeepRobotics M20 MJCF and 57-input/16-output ONNX
policy from `DeepRoboticsLab/sdk_deploy`. The model is a 16-DOF wheel-legged
quadruped; source and license details are recorded in the asset directory.

Simulation sensor settings and the M20 MLS planning envelope are validated
module parameters. The checked-in default profile is:

```text
dimos/robot/deeprobotics/m20/config/mujoco_sim.yaml
```

Edit that file and restart `m20-dan-nav-sim`; opening Python source is not
required. The blueprint loads and validates the YAML at startup. Inline
comments in that file document every checked-in parameter.

| Parameter | Default | Effect |
| --- | --- | --- |
| `enable_color` | `True` | Create and run the RGB renderer |
| `publish_front_image` | `True` | Publish RGB as `color_image` |
| `publish_rear_image` | `False` | Duplicate RGB to the rear topic; no rear renderer exists |
| `width`, `height`, `fps` | `640`, `360`, `10` | Front RGB render size and rate |
| `enable_pointcloud` | `True` | Run synthetic lidar and publish the merged point cloud |
| `pointcloud_scan_pattern` | `airy_hemisphere` | Use a full-azimuth hemispherical MuJoCo ray pattern |
| `pointcloud_fps` | `2` | Synthetic point-cloud rate |
| `pointcloud_width`, `pointcloud_height` | `128`, `96` | Reduced azimuth samples and Airy vertical channels per lidar |
| `pointcloud_min_range_m` | `0.1` | Official Airy decoder minimum range |
| `pointcloud_max_range_m` | `10` | Maximum retained depth hit distance for the office scene |
| `pointcloud_camera_names` | front, rear | MuJoCo mount frames used as ray origins |
| `pointcloud_geom_groups` | `[0, 1]` | MuJoCo geometry groups visible to lidar rays |
| `pointcloud_fov_deg` | `90` | Official Airy vertical FOV; azimuth is 360 degrees |
| `pointcloud_voxel_size` | `0.05` | Open3D downsampling resolution in metres |

The `mlsplannernative` section keeps the planner envelope consistent with the
official M20 MJCF used by this simulation:

| Parameter | Value | Basis |
| --- | --- | --- |
| `robot_height` | `0.70 m` | M20 standing model height plus vertical margin |
| `wall_clearance_m` | `0.50 m` | M20 body and wheel radius plus lateral margin |

The real `m20-dan-nav` blueprint continues to use its separate M20 envelope
(`1.00 m` height and `0.55 m` hard wall clearance). The remaining mapping,
planner cost, and controller parameters are intentionally shared for now.

The generic defaults preserve the legacy G1/Go2 visible groups `(0, 1, 2)`.
The M20 profile limits lidar raycasts to groups `(0, 1)`. The imported
M20 visual geometry is in group `2` and its collision geometry is rendered in
group `3`, so neither is scanned by the synthetic lidars. Including
either group makes MLS inflate robot points into an obstacle around its own
start pose. Keep `publish_rear_image=false` unless duplicate front data is
intentionally required.

The Airy profile is based on RoboSense's
[official 360 x 90-degree product specification](https://www.robosense.ai/en/IncrementalComponents/Airy)
and
[official decoder](https://github.com/RoboSense-LiDAR/rs_driver/blob/897b14d3bdb6186a75df27ba51b65b5bd5557723/src/rs_driver/driver/decoder/decoder_RSAIRY.hpp).
The driver confirms 48/96/192 modes, defaults to 96 channels, accepts distances
from 0.1 to 60 m at 0.005 m resolution, and loads 96 vertical plus 96 horizontal
calibration angles from DIFOP. No official Airy MuJoCo/Gazebo configuration or
M20 mounting transforms are published. The simulator therefore uses uniform
elevation and azimuth samples, limits range to 10 m, and publishes at 2 Hz. It
does not reproduce factory beam calibration, full point rate, scan timing,
noise, blind zones, occlusion, or motion distortion.

The low-load profile was measured on the same ARM VM with headless EGL MuJoCo
and the native Rerun viewer using software-rendered `llvmpipe`:

| Profile | Merged points/frame | Lidar receive rate | Front RGB receive rate |
| --- | ---: | ---: | ---: |
| 192 x 96 per Airy | 26,000-27,500 | 1.6-2.0 Hz | 7-8 FPS |
| 128 x 96 per Airy | 19,000-19,700 | 1.77-1.99 Hz | 7.1-8.1 FPS |

The 128-sample profile reduces displayed point volume by roughly 27% while
preserving 96 vertical channels, front/rear coverage, and successful goal
navigation. RGB remains limited mainly by the VM's software-rendered viewer.

The checked-in default profile uses YAML so it can carry comments. An alternate
runtime config selected with `--config` still uses the DimOS JSON config format,
and one-off values can be overridden without editing either file:

```bash
dimos run m20-dan-nav-sim --config /path/to/custom.json
dimos run m20-dan-nav-sim \
    --option m20mujocosimconnection.pointcloud_fps=1.0
```

The simple-nav profile also enables candidate-validator shadow metrics. Every
plan records raw and fractional-candidate clearance, unknown exposure, path
length, cumulative turn, mean cost, failure reason, and selected alpha. Shadow
mode evaluates diagnostics only; the existing `raw_mean_cost + 2.0` gate still
selects the controller path.

Use `m20-dan-nav` for the real M20 connection. The simulation blueprint does
not include `M20Connection`, so starting it cannot send commands to the robot.

In an SSH session without `DISPLAY`, the current WD Rerun websocket path may
print a native-viewer `winit` warning. The simulation is healthy when the CLI
reports all modules started and the health check passes. `dimos stop` may also
escalate after its graceful timeout; verify that MuJoCo, MLS, voxel processes
and ports `7779`, `3030`, `9877`, and `9878` are gone before restarting.
