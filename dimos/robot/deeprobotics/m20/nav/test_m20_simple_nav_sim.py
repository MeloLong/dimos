# Copyright 2026 Dimensional Inc.
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

import yaml

from dimos.mapping.costmapper import CostMapper
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos.robot.deeprobotics.m20.blueprints.basic import (
    m20_rerun_blueprint,
    m20_sim_rerun_blueprint,
)
from dimos.robot.deeprobotics.m20.connection import M20Connection
from dimos.robot.deeprobotics.m20.mujoco_sim import M20MujocoSimConfig, M20MujocoSimConnection
from dimos.robot.deeprobotics.m20.nav.m20_simple_nav import (
    M20_MUJOCO_SIM_CONFIG_PATH,
    m20_simple_nav,
    m20_simple_nav_sim,
)
from dimos.robot.deeprobotics.m20.nav.moving_obstacle import (
    M20MovingObstacle,
    M20MovingObstacleConfig,
)
from dimos.visualization.rerun.bridge import RerunBridgeModule


def _modules(blueprint):
    return {atom.module for atom in blueprint.blueprints}


def test_simple_nav_real_and_sim_connections_are_isolated() -> None:
    real_modules = _modules(m20_simple_nav)
    sim_modules = _modules(m20_simple_nav_sim)

    assert M20Connection in real_modules
    assert M20MujocoSimConnection not in real_modules
    assert M20MujocoSimConnection in sim_modules
    assert M20Connection not in sim_modules
    assert M20MovingObstacle in sim_modules
    assert M20MovingObstacle not in real_modules


def test_simple_nav_sim_feeds_m20_slam_topics() -> None:
    remappings = m20_simple_nav_sim.remapping_map

    assert remappings[(M20MujocoSimConnection.name, "slam_odom")] == "dimos/slam_odom"
    assert (
        remappings[(M20MujocoSimConnection.name, "slam_aligned_points")]
        == "dimos/slam_aligned_points"
    )
    assert remappings[(M20MovingObstacle.name, "odometry")] == "dimos/slam_odom"


def test_simple_nav_sim_uses_m20_envelope_from_yaml() -> None:
    payload = yaml.safe_load(M20_MUJOCO_SIM_CONFIG_PATH.read_text(encoding="utf-8"))
    clearance = payload["mlsplannernative"]["wall_clearance_m"]
    height = payload["mlsplannernative"]["robot_height"]

    cost_mapper = next(atom for atom in m20_simple_nav_sim.blueprints if atom.module is CostMapper)
    planner = next(
        atom for atom in m20_simple_nav_sim.blueprints if atom.module is ReplanningAStarPlanner
    )

    assert cost_mapper.kwargs["config"].can_pass_under == height
    assert cost_mapper.kwargs["initial_safe_radius_meters"] == clearance
    assert planner.kwargs["robot_width"] == clearance * 2
    assert planner.kwargs["robot_rotation_diameter"] == clearance * 2
    assert m20_simple_nav_sim.global_config_overrides["robot_model"] == "deeprobotics_m20"


def test_simple_nav_sim_loads_raw_path_debug_switch() -> None:
    payload = yaml.safe_load(M20_MUJOCO_SIM_CONFIG_PATH.read_text(encoding="utf-8"))
    planner = next(
        atom for atom in m20_simple_nav_sim.blueprints if atom.module is ReplanningAStarPlanner
    )

    assert (
        planner.kwargs["publish_raw_path"] == payload["replanningastarplanner"]["publish_raw_path"]
    )


def test_simple_nav_sim_loads_constrained_smoothing_profile() -> None:
    payload = yaml.safe_load(M20_MUJOCO_SIM_CONFIG_PATH.read_text(encoding="utf-8"))
    values = payload["replanningastarplanner"]
    planner = next(
        atom for atom in m20_simple_nav_sim.blueprints if atom.module is ReplanningAStarPlanner
    )

    assert values["constrained_path_smoothing_enabled"] is True
    assert values["path_smoothing_validator_shadow_enabled"] is True
    assert values["path_smoothing_physical_validator_shadow_enabled"] is True
    assert values["path_smoothing_physical_validator_authoritative_enabled"] is False
    assert values["path_smoothing_physical_validator_max_clearance_loss_m"] == 0.025
    assert values["path_smoothing_physical_validator_max_unknown_length_increase_m"] == 0.0
    for name, value in values.items():
        assert planner.kwargs[name] == value


def test_simple_nav_sim_loads_checked_in_sensor_profile() -> None:
    payload = yaml.safe_load(M20_MUJOCO_SIM_CONFIG_PATH.read_text(encoding="utf-8"))
    values = payload["m20mujocosimconnection"]
    expected = M20MujocoSimConfig.model_validate(values).model_dump(include=set(values))
    simulator = next(
        atom for atom in m20_simple_nav_sim.blueprints if atom.module is M20MujocoSimConnection
    )

    assert simulator.kwargs == expected
    assert simulator.kwargs["person_collision_enabled"] is False
    assert simulator.kwargs["publish_front_image"] is True
    assert simulator.kwargs["publish_rear_image"] is False
    assert simulator.kwargs["fps"] == 8.0
    assert simulator.kwargs["pointcloud_camera_names"] == (
        "lidar_front_camera",
        "lidar_rear_camera",
    )
    assert simulator.kwargs["pointcloud_scan_pattern"] == "airy_hemisphere"
    assert simulator.kwargs["pointcloud_width"] == 64
    assert simulator.kwargs["pointcloud_height"] == 96
    assert simulator.kwargs["pointcloud_fov_deg"] == 90.0
    assert simulator.kwargs["pointcloud_min_range_m"] == 0.1
    assert simulator.kwargs["pointcloud_max_range_m"] == 10.0


def test_simple_nav_sim_uses_low_load_rerun_profile() -> None:
    bridge = next(
        atom for atom in m20_simple_nav_sim.blueprints if atom.module is RerunBridgeModule
    )

    assert bridge.kwargs["blueprint"] is m20_sim_rerun_blueprint
    assert bridge.kwargs["max_hz"]["world/color_image"] == 0
    assert bridge.kwargs["max_hz"]["world/slam_aligned_points"] == 2.0
    assert bridge.kwargs["debug_low_fps_warn"]["world/color_image"] == 7.0
    assert bridge.kwargs["debug_low_fps_warn"]["world/slam_aligned_points"] == 1.8
    assert "world/slam_aligned_points" in bridge.kwargs["visual_override"]
    assert "world/local_map" in bridge.kwargs["visual_override"]
    assert "world/global_map" in bridge.kwargs["visual_override"]


def test_rerun_blueprints_match_available_cameras() -> None:
    real = m20_rerun_blueprint().root_container
    simulation = m20_sim_rerun_blueprint().root_container

    real_camera_views, real_3d = real.contents
    sim_camera_views, sim_3d = simulation.contents

    assert [view.name for view in real_camera_views.contents] == [
        "M20 Front",
        "M20 Rear",
    ]
    assert [view.name for view in sim_camera_views.contents] == ["M20 Front"]
    expected_3d_contents = [
        "+ $origin/**",
        "- $origin/color_image",
        "- $origin/color_image_rear",
    ]
    assert real_3d.contents == expected_3d_contents
    assert sim_3d.contents == expected_3d_contents


def test_simple_nav_sim_loads_checked_in_moving_obstacle_profile() -> None:
    payload = yaml.safe_load(M20_MUJOCO_SIM_CONFIG_PATH.read_text(encoding="utf-8"))
    values = payload["m20movingobstacle"]
    expected = M20MovingObstacleConfig.model_validate(values).model_dump(include=set(values))
    obstacle = next(
        atom for atom in m20_simple_nav_sim.blueprints if atom.module is M20MovingObstacle
    )

    assert obstacle.kwargs == expected
    assert obstacle.kwargs["waypoints"] == M20MovingObstacleConfig().waypoints
    assert obstacle.kwargs["proximity_stop_distance_m"] == 0.9
    assert obstacle.kwargs["proximity_resume_distance_m"] == 1.1
    assert obstacle.kwargs["proximity_pause_s"] == 1.0
