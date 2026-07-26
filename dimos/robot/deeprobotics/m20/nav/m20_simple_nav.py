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

import yaml

from dimos.core.coordination.blueprints import autoconnect
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.pointclouds.occupancy import HeightCostConfig
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.nav_3d.mls_planner.mls_planner_native import MLSPlannerNativeConfig
from dimos.navigation.replanning_a_star.module import (
    ReplanningAStarPlanner,
    ReplanningAStarPlannerConfig,
)
from dimos.robot.deeprobotics.m20.blueprints.basic import build_m20_rerun, m20
from dimos.robot.deeprobotics.m20.mujoco_sim import (
    M20MujocoSimConfig,
    M20MujocoSimConnection,
)
from dimos.robot.deeprobotics.m20.nav.moving_obstacle import (
    M20MovingObstacle,
    M20MovingObstacleConfig,
)
from dimos.robot.deeprobotics.m20.tf import M20TF

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


def _load_m20_mujoco_sim_config() -> tuple[
    dict[str, Any], dict[str, Any], dict[str, Any], dict[str, Any]
]:
    payload = yaml.safe_load(M20_MUJOCO_SIM_CONFIG_PATH.read_text(encoding="utf-8"))
    connection_values = payload["m20mujocosimconnection"]
    obstacle_values = payload["m20movingobstacle"]
    envelope_values = payload["mlsplannernative"]
    planner_values = payload["replanningastarplanner"]
    connection_config = M20MujocoSimConfig.model_validate(connection_values)
    obstacle_config = M20MovingObstacleConfig.model_validate(obstacle_values)
    envelope_config = MLSPlannerNativeConfig.model_validate(envelope_values)
    planner_config = ReplanningAStarPlannerConfig.model_validate(planner_values)
    return (
        connection_config.model_dump(include=set(connection_values)),
        obstacle_config.model_dump(include=set(obstacle_values)),
        envelope_config.model_dump(include=set(envelope_values)),
        planner_config.model_dump(include=set(planner_values)),
    )


(
    M20_MUJOCO_SIM_CONFIG,
    M20_MOVING_OBSTACLE_CONFIG,
    M20_MUJOCO_ENVELOPE,
    M20_SIM_PLANNER_CONFIG,
) = _load_m20_mujoco_sim_config()

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

m20_simple_nav = autoconnect(
    m20,
    _m20_slam_ray_tracer,
    CostMapper.blueprint(
        config=HeightCostConfig(
            resolution=voxel_size,
            can_pass_under=m20_overhead_clearance,
            can_climb=m20_max_step_height,
            ignore_noise=0.08,
            smoothing=1.5,
            min_gradient_neighbors=2,
            ignore_overhead_only=True,
        ),
        initial_safe_radius_meters=m20_width_clearance + m20_safe_radius_margin,
    ),
    ReplanningAStarPlanner.blueprint(
        robot_width=m20_width_clearance,
        robot_rotation_diameter=m20_rotation_diameter,
    ).remappings([(ReplanningAStarPlanner, "odometry", "dimos/slam_odom")]),
    MovementManager.blueprint(),
).global_config(n_workers=10, robot_model="m20")


# Keep the official MuJoCo model envelope separate from the complete real-robot
# envelope, which also accounts for onboard sensor hardware.
_m20_sim_clearance = M20_MUJOCO_ENVELOPE["wall_clearance_m"]
_m20_sim_height = M20_MUJOCO_ENVELOPE["robot_height"]
_m20_sim_rerun = build_m20_rerun(simulation=True)

m20_simple_nav_sim = autoconnect(
    _m20_sim_rerun,
    _m20_slam_ray_tracer,
    CostMapper.blueprint(
        config=HeightCostConfig(
            resolution=voxel_size,
            can_pass_under=_m20_sim_height,
            can_climb=m20_max_step_height,
            ignore_noise=0.08,
            smoothing=1.5,
            min_gradient_neighbors=2,
            ignore_overhead_only=True,
        ),
        initial_safe_radius_meters=_m20_sim_clearance,
    ),
    ReplanningAStarPlanner.blueprint(
        robot_width=_m20_sim_clearance * 2,
        robot_rotation_diameter=_m20_sim_clearance * 2,
        **M20_SIM_PLANNER_CONFIG,
    ).remappings([(ReplanningAStarPlanner, "odometry", "dimos/slam_odom")]),
    MovementManager.blueprint(),
    M20MujocoSimConnection.blueprint(**M20_MUJOCO_SIM_CONFIG).remappings(
        [
            (M20MujocoSimConnection, "slam_odom", "dimos/slam_odom"),
            (M20MujocoSimConnection, "slam_aligned_points", "dimos/slam_aligned_points"),
        ]
    ),
    M20MovingObstacle.blueprint(**M20_MOVING_OBSTACLE_CONFIG).remappings(
        [(M20MovingObstacle, "odometry", "dimos/slam_odom")]
    ),
    M20TF.blueprint().remappings([(M20TF, "odometry", "dimos/slam_odom")]),
).global_config(
    n_workers=11,
    robot_model="deeprobotics_m20",
    robot_width=_m20_sim_clearance * 2,
    robot_rotation_diameter=_m20_sim_clearance * 2,
    simulation="mujoco",
)
