# M20 MuJoCo Runtime And Configuration Reference

This document is the operational reference for the M20 MuJoCo navigation
simulation. Start with the [M20 integration README](/dimos/robot/deeprobotics/m20/README.md) for asset
provenance, installation, validation evidence, and acceptance limits.

## Blueprint Selection

| Blueprint | Use it to test | Composition |
| --- | --- | --- |
| `m20-simple-nav-sim` | A* planning, constrained path smoothing, obstacle-triggered replanning, and rotate-then-drive tracking | `CostMapper -> ReplanningAStarPlanner -> LocalPlanner/PController` |
| `m20-dan-nav-sim` | MLS global planning, DAN local planning, and holonomic trajectory control | `MLSPlannerNative -> DanLocalPlanner -> DanHolonomicTC` |

Both blueprints use the official DeepRobotics M20 MJCF and ONNX policy. They
publish `dimos/slam_odom`, `dimos/slam_aligned_points`, and front
`color_image`, and consume `cmd_vel`. They run MuJoCo headlessly and do not
instantiate the physical `M20Connection`.

`m20-simple-nav-sim` additionally instantiates `M20MovingObstacle`. The module
moves a person-shaped mocap body through the scene. Its proximity behavior
pauses and redirects the person; it is not a robot-side stop function.

## Clean Startup

Always stop the earlier run before starting a new test. Stale mapping, Viewer,
or simulator processes can consume CPU and retain ports.

```sh skip
cd /path/to/dimos
uv run --no-sync dimos stop
```

For an interactive Simple Nav run with a visible native Rerun Viewer:

```sh skip
MUJOCO_GL=egl uv run --no-sync dimos --rerun-open native run m20-simple-nav-sim
```

For a bounded headless smoke test:

```sh skip
timeout --signal=INT --kill-after=10s 30s \
  env MUJOCO_GL=egl uv run --no-sync dimos --rerun-open none \
  run m20-simple-nav-sim
```

For DAN navigation:

```sh skip
MUJOCO_GL=egl uv run --no-sync dimos --rerun-open native run m20-dan-nav-sim
```

If an SSH shell has no usable desktop environment, use `--rerun-open none` and
launch the Viewer from a desktop terminal:

```sh skip
uv run --no-sync dimos-viewer --connect rerun+http://127.0.0.1:9877/proxy
```

The simulation Rerun layout contains one `M20 Front` 2D view and one 3D view.
`color_image` and `color_image_rear` are explicitly excluded from the 3D view;
a red image entity under 3D indicates an old layout, not a corrupt image.

## Goal And Motion Checks

For every change to control, sensing, or planning:

1. Let the robot settle and confirm odometry, RGB, merged point cloud, local
   map, global map, and costmap update.
2. Send a short forward goal in open space and wait for `goal_reached`.
3. Send a goal that requires initial rotation and observe the transition to
   `path_following`.
4. Check normal teleop in forward, reverse, positive/negative yaw, and lateral
   directions. Treat lateral and signed-yaw magnitude as diagnostic only; they
   are not calibrated acceptance axes.
5. Stop the run and confirm no simulator, mapping, planner, or Viewer process
   remains before repeating the test.

The checked-in simulation yaw adapter multiplies `angular.z` by `2.0` and caps
it at `1.6 rad/s`. Normal Viewer teleop (`0.8 rad/s`) therefore reaches the cap;
Shift fast mode remains capped. Simple Nav's `0.55 rad/s` tracking limit becomes
`1.1 rad/s`. This changes only commands sent to the simulation and does not
modify the official ONNX or physical-robot settings.

## Parameter Reference

The source of truth is
[`config/mujoco_sim.yaml`](/dimos/robot/deeprobotics/m20/config/mujoco_sim.yaml). The comments in that
file describe range constraints and tuning tradeoffs. Restart DimOS after
editing it.

### RGB And Synthetic Lidar

Section: `m20mujocosimconnection`. Used by both simulation blueprints.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `enable_color` | `true` | Enable RGB rendering; must remain true if either image topic is published |
| `color_camera_name` | `head_camera` | MJCF camera used for RGB |
| `color_frame_id` | `camera_optical` | Frame ID attached to simulation image metadata |
| `width`, `height` | `640`, `360` | Front RGB dimensions |
| `fps` | `8.0` | RGB source publication target |
| `color_fov_deg` | `45.0` | Vertical RGB field of view |
| `publish_front_image` | `true` | Publish `color_image` |
| `publish_rear_image` | `false` | Do not duplicate the front renderer on the rear topic |
| `yaw_command_scale` | `2.0` | Simulation-only multiplier for `angular.z` |
| `yaw_command_limit` | `1.6` | Absolute simulation yaw cap in rad/s |
| `enable_pointcloud` | `true` | Enable synthetic lidar and merged point-cloud publication |
| `pointcloud_scan_pattern` | `airy_hemisphere` | Use camera-forward 180 x 90-degree sectors |
| `pointcloud_fps` | `2.0` | Merged point-cloud target rate |
| `pointcloud_width` | `64` | Azimuth samples per Airy sector |
| `pointcloud_height` | `96` | Vertical channels per Airy unit |
| `pointcloud_camera_names` | front, rear | MJCF ray origins; their frames face opposite directions |
| `pointcloud_geom_groups` | `[0, 1]` | Scene groups visible to rays; robot groups 2 and 3 remain excluded |
| `person_collision_enabled` | `false` | Keep the mocap person visible to rays without physical contact |
| `pointcloud_fov_deg` | `90.0` | Vertical FOV; azimuth is fixed to 180 degrees by this scan pattern |
| `pointcloud_min_range_m` | `0.1` | Minimum retained ray hit |
| `pointcloud_max_range_m` | `10.0` | Maximum retained ray hit in the office profile |
| `pointcloud_voxel_size` | `0.05` | Navigation point-cloud voxel downsampling in metres |

The real RoboSense Airy specification is 360 x 90 degrees and the decoder
supports up to 60 m. Two full-azimuth synthetic units would produce duplicate
coverage because robot geometry is excluded from raycasts, so the current
model uses outward front/rear sectors. The profile does not reproduce DIFOP
beam calibration, production point rate, scan timing, noise, blind zones,
occlusion, or motion distortion.

### Moving Person

Section: `m20movingobstacle`. Used only by `m20-simple-nav-sim`.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `enabled` | `true` | Start the simulated person |
| `seed` | `20` | Reproducible pseudo-random route choices |
| `speed_mps` | `0.15` | Person ground speed |
| `update_hz` | `20.0` | Mocap pose publication rate |
| `initial_waypoint_index` | `0` | Initial node in the checked-in office route |
| `z_m` | `0.0` | Ground-level mocap height |
| `proximity_stop_distance_m` | `0.9` | Pause distance from robot centre |
| `proximity_resume_distance_m` | `1.1` | Hysteresis distance for normal walking to resume |
| `proximity_pause_s` | `1.0` | Pause before selecting a retreat edge |
| `waypoints` | Five office points | Validated route graph used for adjacent-edge choices |

The normal run needs no `enabled=true` CLI override. YAML cannot start the
module by itself; the root Simple Nav blueprint creates and wires it.

### Planning Envelope

Section: `mlsplannernative`. Used by both simulation blueprints.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `robot_height` | `0.7` | Standing MJCF height plus simulation margin |
| `wall_clearance_m` | `0.5` | Radial body/wheel clearance from walls and map edges |

The physical `m20-dan-nav` profile intentionally uses a larger 1.00 m vertical
envelope and 0.55 m wall clearance for the complete hardware. Do not copy the
simulation values into real-robot safety configuration.

### Simple Nav Path Processing

Section: `replanningastarplanner`. Used only by `m20-simple-nav-sim`.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `publish_raw_path` | `true` | Publish grid A* output for Rerun comparison |
| `constrained_path_smoothing_enabled` | `true` | Use bounded, costmap-aware smoothing |
| `path_smoothing_iterations` | `40` | Maximum iterative smoothing steps |
| `path_smoothing_data_weight` | `0.02` | Attraction to the raw A* path |
| `path_smoothing_smoothness_weight` | `0.45` | Neighbour/Laplacian smoothing strength |
| `path_smoothing_max_deviation_m` | `0.10` | Maximum displacement from matching raw points |
| `path_smoothing_collision_sample_spacing_m` | `0.05` | Costmap sample spacing along candidate segments |
| `path_smoothing_max_cost_increase` | `2.0` | Maximum accepted mean-cost increase |
| `path_smoothing_backtracking_factor` | `0.5` | Retained smoothing fraction after a failed candidate |
| `path_smoothing_max_backtracking_steps` | `3` | Reduced-fraction retries after the full candidate |
| `path_smoothing_validator_shadow_enabled` | `true` | Record candidate metrics without changing selection |
| `path_smoothing_physical_validator_shadow_enabled` | `true` | Evaluate Stage 2 physical policy in shadow mode |
| `path_smoothing_physical_validator_authoritative_enabled` | `false` | Keep the physical policy non-authoritative |
| `path_smoothing_physical_validator_max_clearance_loss_m` | `0.025` | Provisional maximum candidate clearance loss |
| `path_smoothing_physical_validator_max_unknown_length_increase_m` | `0.0` | Additional allowed path length in unknown cells |
| `path_resample_spacing_m` | `0.10` | Approximate final controller waypoint spacing |

Shadow validators publish diagnostics only while
`path_smoothing_physical_validator_authoritative_enabled` is false. The legacy
collision/cost/backtracking decision remains authoritative.

### Runtime Overrides

One-off values can be changed without editing YAML:

```sh skip
uv run --no-sync dimos --rerun-open none run m20-simple-nav-sim \
  --option m20mujocosimconnection.pointcloud_fps=1.0 \
  --option m20mujocosimconnection.pointcloud_width=32 \
  --option m20movingobstacle.enabled=false
```

An alternate `--config` file uses the DimOS JSON configuration format, not the
M20 YAML schema:

```sh skip
uv run --no-sync dimos run m20-dan-nav-sim --config /path/to/custom.json
```

## Performance Reference

The final profile was measured on the ARM VM described in the integration
README with EGL MuJoCo and a visible native Viewer rendered through
`llvmpipe`.

| Profile | RGB result | Long-frame behavior | Lidar result |
| --- | --- | --- | --- |
| Earlier 128 x 96 Airy sectors | About 7.56 FPS | Maximum interval 255.7 ms | About 1.8-2.0 Hz |
| Current 64 x 96 Airy sectors | About 7.73 FPS | P95 133.5 ms; maximum 135.6 ms | About 1.8-2.0 Hz |

The change preserves both sensors and all 96 vertical lines while reducing
synthetic azimuth work. Rerun's point cloud is downsampled again for display;
that visual override does not change the point cloud consumed by navigation.
Occasional isolated long frames remain possible during global-map or goal
transitions on software rendering.

A 30-second headless startup on the current profile deployed all 11 modules,
loaded the official ONNX, sustained 7.70-7.91 RGB FPS and 1.98-1.99 lidar Hz,
and exercised moving-person pause, redirect, and resume behavior. SIGINT
shutdown removed all processes and listeners but printed a Python
`resource_tracker` warning for seven shared-memory objects.

A separate 30-second `m20-dan-nav-sim` startup deployed all 12 modules and
started the native voxel mapper, native MLS planner, DAN planner/controller,
official ONNX, and MuJoCo process. Its shutdown also removed all processes and
listeners and reproduced the same shared-memory warning.

## Recording And Replay

### Rerun RRD

Start the recording server before DimOS so it owns port `9877` and receives the
first event:

```sh skip
mkdir -p /public/M20_dimos
uv run --no-sync rerun --serve-grpc --port 9877 \
  --server-memory-limit 15GB \
  --save "/public/M20_dimos/m20_$(date +%Y%m%d_%H%M%S).rrd"
```

In a second terminal:

```sh skip
MUJOCO_GL=egl uv run --no-sync dimos --rerun-open none run m20-simple-nav-sim
```

Stop DimOS first, then stop the Rerun server so the file is finalized. Verify
and replay it with:

```sh skip
uv run --no-sync rerun rrd verify /public/M20_dimos/run.rrd
uv run --no-sync rerun /public/M20_dimos/run.rrd --memory-limit 8GB
```

Do not run `uv run /path/file.rrd`; `uv run` expects an executable in that
position. The executable is `rerun`, and the RRD path is its argument.

If DimOS starts first, its bridge claims `9877` without `--save`. A later
recording server cannot bind the same port or recover earlier events. Stop both
and restart them in the documented order.

### SQLite DB

Compose `nav-record` at initial startup:

```sh skip
mkdir -p /public/M20_dimos/db
uv run --no-sync dimos --rerun-open none run m20-simple-nav-sim nav-record \
  --option navrecord.db_path=/public/M20_dimos/db/m20-sim.db
```

The current composition records only connected recorder streams: TF and
`global_map`. It does not automatically capture odometry, raw point clouds, or
RGB. Render a DB to RRD with:

```sh skip
uv run --no-sync dimos mem rerun /path/run.db \
  --out /path/run-from-db.rrd --no-gui
```

The recorder is part of the initial coordinator composition and cannot be
added dynamically to an already-running navigation coordinator. A standalone
M20 recorder with explicit stream mappings would be required for independent
mid-run DB capture. See
[Navigation Recording And Replay](/docs/usage/navigation_recording_replay.md)
for the generic recording model.

## Troubleshooting And Cleanup

| Symptom | Action |
| --- | --- |
| Camera view shows three loading dots | Confirm the simulation-specific front-only Rerun blueprint is active; restart after updating the checkout |
| RGB is slow while maps update | Check whether the Viewer uses `llvmpipe`; reduce display load or provide graphics acceleration before reducing 96 lidar channels |
| Front/rear lidar coverage looks identical | Confirm the current 180-degree sector implementation and opposite MJCF camera frames are present |
| Robot turns only in Viewer fast mode | Confirm `yaw_command_scale: 2.0` and `yaw_command_limit: 1.6`, then restart |
| CLI reports a missing obstacle waypoint | The checkout has an old partial-config bug; use the current YAML and omit the redundant `enabled=true` override |
| Port `9877` is already in use | Stop DimOS and the external recorder; start the recorder first only when saving RRD |
| `dimos stop` times out | Check and terminate stale MuJoCo, MLS, voxel-map, and Viewer processes before relaunching |
| Shutdown prints a shared-memory `resource_tracker` warning | Check for residual processes and ports; the bounded reference run cleaned them up, but graceful shared-memory teardown remains an open lifecycle issue |

Useful listener check:

```sh skip
ss -ltnp | grep -E ':(3030|7779|9877|9878)\b' || true
```

The simulation is healthy in headless mode when all modules start, health
checks pass, streams update, and shutdown removes the coordinator and child
processes. A missing GUI by itself is not a simulator failure.
