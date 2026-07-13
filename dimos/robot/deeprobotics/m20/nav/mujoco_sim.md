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
topics and consumes `cmd_vel`. RGB rendering is disabled in this navigation
profile because the simulated `head_camera` is not an M20 camera and software
EGL rendering is the dominant CPU cost. The rear image topic is also disabled;
the legacy simulator has no rear camera and previously duplicated the front
frame. The simulator still uses the existing Unitree Go1/Go2 model and policy
as a navigation data source. It does not validate DeepRobotics M20 dynamics,
actuators, gait control, or physical limits.

Sensor settings are validated `ModuleConfig` parameters on
`M20MujocoSimConnection`. The checked-in default profile is:

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
| `width`, `height`, `fps` | `640`, `360`, `20` | RGB/depth render size and RGB rate |
| `enable_pointcloud` | `True` | Run depth renderers and publish the synthetic point cloud |
| `pointcloud_fps` | `2` | Synthetic point-cloud rate |
| `pointcloud_camera_names` | front, left, right | MuJoCo cameras used for point-cloud generation |
| `pointcloud_fov_deg` | `160` | Depth projection field of view |
| `pointcloud_voxel_size` | `0.05` | Open3D downsampling resolution in metres |

The generic defaults preserve the existing G1/Go2 simulator behavior. The M20
JSON profile sets `enable_color=false` and disables both image publications.
To test the synthetic front image, set `enable_color=true` and
`publish_front_image=true` in the JSON; leave `publish_rear_image=false` unless
duplicate data is intentionally required.

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
