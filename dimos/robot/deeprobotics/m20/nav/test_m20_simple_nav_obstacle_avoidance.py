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

from pathlib import Path

from dimos.navigation.obstacle_avoidance.module import (
    ObstacleAvoidance,
    ObstacleAvoidanceConfig,
)
from dimos.robot.get_all_blueprints import get_by_name
from dimos.utils.config import load_config_mapping

M20_SIMPLE_NAV_CONFIG_PATH = Path(__file__).resolve().parents[1] / "config/m20_simple_nav.yaml"

EXPECTED_DYNAMIC_LOOKAHEAD_CONFIG = {
    "obstacle_lookahead_min_distance_m": 3.0,
    "obstacle_lookahead_time_s": 0.0,
    "obstacle_lookahead_max_distance_m": 3.0,
}

EXPECTED_OBSTACLE_AVOIDANCE_CONFIG = {
    "enabled": False,
    "stop_distance_m": 1.2,
    "resume_distance_m": 1.4,
    "reaction_time_s": 0.5,
    "braking_deceleration_m_s2": 0.6,
    "stop_safety_margin_m": 0.2,
    "lateral_margin_m": 0.1,
    "clear_costmap_count": 2,
    "costmap_timeout_s": 1.5,
    "odometry_timeout_s": 0.5,
}


def _obstacle_avoidance_config_from_blueprint() -> ObstacleAvoidanceConfig:
    blueprint = get_by_name("m20-simple-nav")
    atom = next(atom for atom in blueprint.blueprints if atom.module is ObstacleAvoidance)
    return ObstacleAvoidanceConfig.model_validate(atom.kwargs)


def test_m20_simple_nav_obstacle_avoidance_profile_matches_blueprint() -> None:
    payload = load_config_mapping(M20_SIMPLE_NAV_CONFIG_PATH)

    assert payload["obstacleavoidance"] == EXPECTED_OBSTACLE_AVOIDANCE_CONFIG
    assert {
        key: payload["replanningastarplanner"][key] for key in EXPECTED_DYNAMIC_LOOKAHEAD_CONFIG
    } == EXPECTED_DYNAMIC_LOOKAHEAD_CONFIG

    obstacle_avoidance = _obstacle_avoidance_config_from_blueprint()
    assert (
        obstacle_avoidance.model_dump(include=set(EXPECTED_OBSTACLE_AVOIDANCE_CONFIG))
        == EXPECTED_OBSTACLE_AVOIDANCE_CONFIG
    )


def test_m20_simple_nav_obstacle_avoidance_is_disabled_and_wired() -> None:
    blueprint = get_by_name("m20-simple-nav")
    modules = {atom.module for atom in blueprint.blueprints}

    assert ObstacleAvoidance in modules
    assert _obstacle_avoidance_config_from_blueprint().enabled is False
    assert blueprint.remapping_map[("replanningastarplanner", "nav_cmd_vel")] == ("raw_nav_cmd_vel")
    assert blueprint.remapping_map[("obstacleavoidance", "odometry")] == "slam_odom"
    assert blueprint.global_config_overrides["robot_width"] == 0.45
    assert blueprint.global_config_overrides["robot_rotation_diameter"] == 1.2
