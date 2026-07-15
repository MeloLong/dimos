#!/usr/bin/env python3
# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Simple M20 navigation stack.

This blueprint consumes the M20 onboard SLAM/LIO outputs that are bridged into
DimOS as ``slam_aligned_points`` and ``slam_odom``. It does not subscribe to the
front/rear raw lidar topics directly.
"""

from pathlib import Path
from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.navigation.dannav.holonomic_tc.module import DanHolonomicTC
from dimos.navigation.dannav.local_planner.module import DanLocalPlanner
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.nav_3d.mls_planner.goal_relay import GoalRelay
from dimos.robot.deeprobotics.m20.blueprints.basic import (
    _node_edges_on_surface,
    m20_rerun_blueprint,
)
from dimos.robot.deeprobotics.m20.connection import M20Connection
from dimos.robot.deeprobotics.m20.nav.fixed_forward_path_planner import FixedForwardPathPlanner

# from dimos.navigation.nav_3d.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.robot.deeprobotics.m20.nav.odom2posestamped import OdomToPoseStamped
from dimos.robot.deeprobotics.m20.tf import M20TF
from dimos.visualization.vis_module import vis_module

voxel_size = 0.05
m20_width_clearance = 0.45
# SLAM odometry is the robot head pose. The head is about 1.3 m above the
# bottom, so overhead clearance must reflect the full bottom-to-head height.
m20_height_clearance = 0.8
m20_overhead_safety_margin = 0.2
m20_overhead_clearance = m20_height_clearance + m20_overhead_safety_margin
m20_max_step_height = 0.15
m20_rotation_diameter = 1.2
m20_safe_radius_margin = 0.1
map_save_dir = Path(__file__).resolve().parent / "map_save"
map_save_path = map_save_dir / "m20_accumulated_map.pcd"

_m20_slam_ray_tracer = RayTracingVoxelMap.blueprint(
    voxel_size=voxel_size,
    max_range=99.0,
    shadow_depth=0.1,
    min_health=-1,
    max_health=5,
    emit_every=2,
    global_emit_every=10,
    # M20's onboard SLAM publishes this cloud already registered in map frame.
    registered_clouds=True,
).remappings(
    [
        (RayTracingVoxelMap, "lidar", "dimos/slam_aligned_points"),
        (RayTracingVoxelMap, "odometry", "dimos/slam_odom"),
    ]
)

# _m20_pointcloud_map_save = PointCloudMapSave.blueprint(
#     translation_threshold_m=0.5,
#     rotation_threshold_rad=math.radians(15.0),
#     voxel_size=voxel_size,
#     save_path=str(map_save_path),
# ).remappings(
#     [
#         (PointCloudMapSave, "lidar", "dimos/slam_aligned_points"),
#         (PointCloudMapSave, "odometry", "dimos/slam_odom"),
#         (PointCloudMapSave, "global_map", "dimos/m20_saved_pointcloud_map"),
#     ]
# )


def _render_path(msg: Any) -> Any:
    # Keep the last displayed route when the controller receives an empty stop path.
    if len(msg.poses) == 0:
        return None
    return msg


_m20_nav_rerun_config = {
    "blueprint": m20_rerun_blueprint,
    "memory_limit": "10GB",
    "max_hz": {
        "world/color_image": 0,
        "world/color_image_rear": 0,
        "world/global_map": 1.0,
        "world/local_map": 2.0,
    },
    "visual_override": {
        "world/node_edges": _node_edges_on_surface,
        "world/planner_path": None,
        "world/path": _render_path,
    },
}

_m20_simple_nav_base = autoconnect(
    vis_module(viewer_backend=global_config.viewer, rerun_config=_m20_nav_rerun_config),
    M20Connection.blueprint(),
    M20TF.blueprint().remappings([(M20TF, "odometry", "dimos/slam_odom")]),
)


m20_dan_nav = autoconnect(
    _m20_simple_nav_base,
    _m20_slam_ray_tracer,
    # CostMapper.blueprint(
    #     config=HeightCostConfig(
    #         resolution=voxel_size,
    #         can_pass_under=m20_overhead_clearance,
    #         can_climb=m20_max_step_height,
    #         ignore_noise=0.08,
    #         smoothing=1.5,
    #         min_gradient_neighbors=2,
    #         ignore_overhead_only=True,
    #     ),
    #     initial_safe_radius_meters=m20_width_clearance + m20_safe_radius_margin,
    # ),
    FixedForwardPathPlanner.blueprint(
        path_length_m=4.0,
        sample_spacing_m=0.2,
        corridor_radius_m=m20_width_clearance + m20_safe_radius_margin,
        min_relative_z_m=-0.2,
        max_relative_z_m=m20_overhead_clearance,
    ).remappings(
        [
            (FixedForwardPathPlanner, "path", "planner_path"),
            # Keep the planner contract MLS-compatible while driving only from
            # local_map + start/goal. The accumulated global map is not used.
            (FixedForwardPathPlanner, "global_map", "global_map_unused"),
        ]
    ),
    # MLSPlannerNative.blueprint(
    #     world_frame="map",
    #     voxel_size=voxel_size,
    #     robot_height=1.2,
    #     wall_clearance_m=0.2,
    #     wall_buffer_m=0.75,
    #     wall_buffer_weight=100.0,
    #     step_threshold_m=0.16,
    #     step_penalty_weight=1.0,
    #     viz_publish_hz=0.0,
    # ).remappings(
    #     [
    #         (MLSPlannerNative, "path", "planner_path"),
    #         # Use the incremental local_map + region_bounds pair from ray tracing.
    #         (MLSPlannerNative, "global_map", "global_map_unused"),
    #     ]
    # ),
    OdomToPoseStamped.blueprint().remappings(
        [
            (OdomToPoseStamped, "odometry", "dimos/slam_odom"),
            (OdomToPoseStamped, "pose", "odom"),
        ]
    ),
    GoalRelay.blueprint().remappings(
        [
            (GoalRelay, "odometry", "dimos/slam_odom"),
        ]
    ),
    # Keep this as pass-through for the fixed test paths; resampling only densifies
    # the straight segment before DanHolonomicTC consumes it.
    DanLocalPlanner.blueprint(
        lock_replan=0.0,
        resample_spacing_m=0.1,
    ),
    DanHolonomicTC.blueprint(run_profile="walk"),
    MovementManager.blueprint(),
).global_config(n_workers=10, robot_model="m20")
