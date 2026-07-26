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

"""DAN M20 navigation stack.

This blueprint consumes the M20 onboard SLAM/LIO outputs that are bridged into
DimOS as ``slam_aligned_points`` and ``slam_odom``. It does not subscribe to the
front/rear raw lidar topics directly.
"""

from pathlib import Path
from typing import Any

import yaml

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.navigation.dannav.holonomic_tc.module import DanHolonomicTC
from dimos.navigation.dannav.local_planner.module import DanLocalPlanner
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.nav_3d.mls_planner.goal_relay import GoalRelay
from dimos.navigation.nav_3d.mls_planner.mls_planner_native import (
    MLSPlannerNative,
    MLSPlannerNativeConfig,
)
from dimos.robot.deeprobotics.m20.blueprints.basic import (
    _node_edges_on_surface,
    m20_rerun_blueprint,
)
from dimos.robot.deeprobotics.m20.connection import M20Connection
from dimos.robot.deeprobotics.m20.mujoco_sim import (
    M20MujocoSimConfig,
    M20MujocoSimConnection,
)
from dimos.robot.deeprobotics.m20.nav.odom2posestamped import OdomToPoseStamped
from dimos.robot.deeprobotics.m20.tf import M20TF
from dimos.visualization.rerun.bridge import RerunBridgeModule
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer
from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule

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
M20_MUJOCO_SIM_CONFIG_PATH = Path(__file__).resolve().parents[1] / "config/mujoco_sim.yaml"


def _load_m20_mujoco_sim_config() -> tuple[dict[str, Any], dict[str, Any]]:
    payload = yaml.safe_load(M20_MUJOCO_SIM_CONFIG_PATH.read_text(encoding="utf-8"))
    connection_values = payload["m20mujocosimconnection"]
    planner_values = payload["mlsplannernative"]
    connection_config = M20MujocoSimConfig.model_validate(connection_values)
    planner_config = MLSPlannerNativeConfig.model_validate(planner_values)
    return (
        connection_config.model_dump(include=set(connection_values)),
        planner_config.model_dump(include=set(planner_values)),
    )


M20_MUJOCO_SIM_CONFIG, M20_MUJOCO_MLS_CONFIG = _load_m20_mujoco_sim_config()

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


def _render_path(msg: Any) -> Any:
    # Keep the last displayed route when the controller receives an empty stop path.
    if len(msg.poses) == 0:
        return None
    return msg


_m20_nav_rerun_config = {
    "blueprint": m20_rerun_blueprint,
    "memory_limit": "1GB",
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

_m20_dan_rerun = autoconnect(
    RerunBridgeModule.blueprint(**_m20_nav_rerun_config),
    RerunWebSocketServer.blueprint(),
    WebsocketVisModule.blueprint(),
)


def _mls_planner_blueprint(*, robot_height: float, wall_clearance_m: float) -> Blueprint:
    return MLSPlannerNative.blueprint(
        world_frame="map",
        voxel_size=voxel_size,
        robot_height=robot_height,
        wall_clearance_m=wall_clearance_m,
        wall_buffer_m=0.75,
        wall_buffer_weight=100.0,
        step_threshold_m=m20_max_step_height,
        step_penalty_weight=1.0,
        goal_tolerance=0.3,
        viz_publish_hz=1.0,
    ).remappings(
        [
            (MLSPlannerNative, "path", "planner_path"),
            # Use the incremental local_map + region_bounds pair from ray tracing.
            (MLSPlannerNative, "global_map", "global_map_unused"),
        ]
    )


_m20_dan_nav_core = autoconnect(
    _m20_dan_rerun,
    _m20_slam_ray_tracer,
    _mls_planner_blueprint(
        robot_height=m20_overhead_clearance,
        wall_clearance_m=m20_width_clearance + m20_safe_radius_margin,
    ),
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
    DanLocalPlanner.blueprint(
        lock_replan=1.0,
        resample_spacing_m=0.1,
    ),
    DanHolonomicTC.blueprint(run_profile="walk"),
    MovementManager.blueprint(),
)

_m20_dan_nav_sim_core = autoconnect(
    _m20_dan_nav_core,
    _mls_planner_blueprint(
        robot_height=M20_MUJOCO_MLS_CONFIG["robot_height"],
        wall_clearance_m=M20_MUJOCO_MLS_CONFIG["wall_clearance_m"],
    ),
)

m20_dan_nav = autoconnect(
    _m20_dan_nav_core,
    M20Connection.blueprint(),
    M20TF.blueprint().remappings([(M20TF, "odometry", "dimos/slam_odom")]),
).global_config(n_workers=10, robot_model="m20")

m20_dan_nav_sim = autoconnect(
    _m20_dan_nav_sim_core,
    M20MujocoSimConnection.blueprint(**M20_MUJOCO_SIM_CONFIG).remappings(
        [
            (M20MujocoSimConnection, "slam_odom", "dimos/slam_odom"),
            (
                M20MujocoSimConnection,
                "slam_aligned_points",
                "dimos/slam_aligned_points",
            ),
        ]
    ),
    M20TF.blueprint().remappings([(M20TF, "odometry", "dimos/slam_odom")]),
).global_config(
    n_workers=11,
    robot_model="deeprobotics_m20",
    simulation="mujoco",
)
