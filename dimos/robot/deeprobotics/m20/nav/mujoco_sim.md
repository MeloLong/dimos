# M20 Navigation MuJoCo Test

The Go1-backed MuJoCo adapter can exercise two M20 navigation chains without
opening a MuJoCo window:

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
Go1 envelope from `mujoco_sim.yaml`: 0.50 m height and 0.45 m radial
clearance, resulting in a 0.90 m A* robot width and rotation diameter. It is a
planning/control integration test, not an M20 dynamics validation.

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

The simulation also enables one person-shaped moving obstacle. It reuses the
existing MuJoCo mocap person and `/person_pose` transport, so the obstacle is
visible to RGB and synthetic point clouds. Physical contact is disabled by
default because a prescribed mocap body has effectively infinite mass and can
push over the Go1 instead of testing perception and replanning. A fixed random
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
simulator uses the existing Unitree Go1 model and Go1 ONNX policy as a
navigation data source. It does not validate DeepRobotics M20 dynamics,
actuators, gait control, or physical limits.

Simulation sensor settings and the Go1 MLS planning envelope are validated
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
| `width`, `height`, `fps` | `640`, `360`, `10` | RGB/depth render size and RGB rate |
| `enable_pointcloud` | `True` | Run depth renderers and publish the synthetic point cloud |
| `pointcloud_fps` | `2` | Synthetic point-cloud rate |
| `pointcloud_max_range_m` | `10` | Maximum retained depth hit distance for the office scene |
| `pointcloud_camera_names` | front, left, right | MuJoCo cameras used for point-cloud generation |
| `pointcloud_geom_groups` | `[0, 1]` | MuJoCo geometry groups visible to point-cloud cameras |
| `pointcloud_fov_deg` | `160` | Depth projection field of view |
| `pointcloud_voxel_size` | `0.05` | Open3D downsampling resolution in metres |

The `mlsplannernative` section keeps the planner envelope consistent with the
actual Go1 MJCF used by this simulation:

| Parameter | Value | Basis |
| --- | --- | --- |
| `robot_height` | `0.50 m` | Go1 visual height is about 0.373 m in the nominal standing pose, plus vertical margin |
| `wall_clearance_m` | `0.45 m` | Go1 maximum nominal horizontal radius is about 0.399 m, plus lateral margin |

The real `m20-dan-nav` blueprint continues to use its separate M20 envelope
(`1.00 m` height and `0.55 m` hard wall clearance). The remaining mapping,
planner cost, and controller parameters are intentionally shared for now.

The generic defaults preserve the legacy G1/Go2 visible groups `(0, 1, 2)`.
The M20 profile limits point-cloud rendering to groups `(0, 1)` because the
Go1 visual model is in group `2`. Including group `2` scans the simulated robot
itself, and MLS then inflates those points into an obstacle around its own
start pose. Keep `publish_rear_image=false` unless duplicate front data is
intentionally required.

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
