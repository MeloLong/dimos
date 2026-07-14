# M20 Dan Navigation MuJoCo Test

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
dimos/robot/deeprobotics/m20/config/mujoco_sim.json
```

Edit that file and restart `m20-dan-nav-sim`; opening Python source is not
required. The blueprint loads and validates the JSON at startup.

| Parameter | Default | Effect |
| --- | --- | --- |
| `enable_color` | `True` | Create and run the RGB renderer |
| `publish_front_image` | `True` | Publish RGB as `color_image` |
| `publish_rear_image` | `False` | Duplicate RGB to the rear topic; no rear renderer exists |
| `width`, `height`, `fps` | `640`, `360`, `10` | RGB/depth render size and RGB rate |
| `enable_pointcloud` | `True` | Run depth renderers and publish the synthetic point cloud |
| `pointcloud_fps` | `2` | Synthetic point-cloud rate |
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

An alternate complete config can be selected with `--config`, and one-off
values can be overridden without editing the file:

```bash
dimos run m20-dan-nav-sim --config /path/to/custom.json
dimos run m20-dan-nav-sim \
  --option m20mujocosimconnection.pointcloud_fps=1.0
```

Use `m20-dan-nav` for the real M20 connection. The simulation blueprint does
not include `M20Connection`, so starting it cannot send commands to the robot.

In an SSH session without `DISPLAY`, the current WD Rerun websocket path may
print a native-viewer `winit` warning. The simulation is healthy when the CLI
reports all modules started and the health check passes. `dimos stop` may also
escalate after its graceful timeout; verify that MuJoCo, MLS, voxel processes
and ports `7779`, `3030`, `9877`, and `9878` are gone before restarting.
