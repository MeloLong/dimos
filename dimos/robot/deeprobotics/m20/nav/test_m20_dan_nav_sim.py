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

import json

from pydantic import ValidationError
import pytest
import yaml

from dimos.navigation.nav_3d.mls_planner.mls_planner_native import (
    MLSPlannerNative,
    MLSPlannerNativeConfig,
)
from dimos.robot.cli.dimos import load_config_args
from dimos.robot.deeprobotics.m20.connection import M20Connection
from dimos.robot.deeprobotics.m20.mujoco_sim import (
    M20MujocoSimConfig,
    M20MujocoSimConnection,
)
from dimos.robot.deeprobotics.m20.nav.m20_dan_nav import (
    M20_MUJOCO_SIM_CONFIG_PATH,
    m20_dan_nav,
    m20_dan_nav_sim,
)


def _modules(blueprint):
    return {atom.module for atom in blueprint.blueprints}


def test_real_and_sim_connections_are_isolated() -> None:
    real_modules = _modules(m20_dan_nav)
    sim_modules = _modules(m20_dan_nav_sim)

    assert M20Connection in real_modules
    assert M20MujocoSimConnection not in real_modules
    assert M20MujocoSimConnection in sim_modules
    assert M20Connection not in sim_modules


def test_sim_outputs_feed_wd_slam_topics() -> None:
    assert m20_dan_nav_sim.remapping_map[(M20MujocoSimConnection, "slam_odom")] == "dimos/slam_odom"
    assert (
        m20_dan_nav_sim.remapping_map[(M20MujocoSimConnection, "slam_aligned_points")]
        == "dimos/slam_aligned_points"
    )


def test_real_and_sim_mls_envelopes_are_isolated() -> None:
    real_planner = next(atom for atom in m20_dan_nav.blueprints if atom.module is MLSPlannerNative)
    sim_planner = next(
        atom for atom in m20_dan_nav_sim.blueprints if atom.module is MLSPlannerNative
    )

    assert real_planner.kwargs["robot_height"] == 1.0
    assert real_planner.kwargs["wall_clearance_m"] == 0.55
    assert sim_planner.kwargs["robot_height"] == 0.7
    assert sim_planner.kwargs["wall_clearance_m"] == 0.5
    assert m20_dan_nav_sim.global_config_overrides["robot_model"] == "deeprobotics_m20"


def test_rear_image_is_not_published_by_default() -> None:
    config = M20MujocoSimConfig()

    assert config.enable_color
    assert config.publish_front_image
    assert not config.publish_rear_image


def test_disabling_color_requires_disabling_image_topics() -> None:
    with pytest.raises(ValidationError, match="image publication requires enable_color"):
        M20MujocoSimConfig(enable_color=False)

    config = M20MujocoSimConfig(
        enable_color=False,
        publish_front_image=False,
        publish_rear_image=False,
    )
    assert not config.sensor_config().enable_color


def test_sensor_parameters_are_projected_to_connection_config() -> None:
    config = M20MujocoSimConfig(
        width=320,
        height=180,
        fps=5,
        pointcloud_fps=1,
        pointcloud_voxel_size=0.1,
    )

    sensors = config.sensor_config()
    assert (sensors.width, sensors.height, sensors.fps) == (320, 180, 5)
    assert sensors.pointcloud_fps == 1
    assert sensors.pointcloud_voxel_size == 0.1


def test_m20_navigation_sim_uses_lightweight_sensor_profile() -> None:
    atom = next(
        atom for atom in m20_dan_nav_sim.blueprints if atom.module is M20MujocoSimConnection
    )

    assert atom.kwargs["enable_color"] is True
    assert atom.kwargs["publish_front_image"] is True
    assert atom.kwargs["publish_rear_image"] is False
    assert atom.kwargs["enable_pointcloud"] is True
    assert atom.kwargs["pointcloud_geom_groups"] == (0, 1)


def test_m20_navigation_sim_loads_sensor_profile_from_yaml() -> None:
    payload = yaml.safe_load(M20_MUJOCO_SIM_CONFIG_PATH.read_text(encoding="utf-8"))
    values = payload["m20mujocosimconnection"]
    expected = M20MujocoSimConfig.model_validate(values).model_dump(include=set(values))
    atom = next(
        atom for atom in m20_dan_nav_sim.blueprints if atom.module is M20MujocoSimConnection
    )

    assert atom.kwargs == expected


def test_m20_navigation_sim_loads_planner_profile_from_yaml() -> None:
    payload = yaml.safe_load(M20_MUJOCO_SIM_CONFIG_PATH.read_text(encoding="utf-8"))
    values = payload["mlsplannernative"]
    expected = MLSPlannerNativeConfig.model_validate(values).model_dump(include=set(values))
    atom = next(atom for atom in m20_dan_nav_sim.blueprints if atom.module is MLSPlannerNative)

    assert {key: atom.kwargs[key] for key in values} == expected


def test_partial_cli_config_keeps_checked_in_sensor_defaults(tmp_path) -> None:
    override_path = tmp_path / "override.json"
    override_path.write_text(
        json.dumps({"m20mujocosimconnection": {"pointcloud_fps": 1.0}}),
        encoding="utf-8",
    )
    overrides = load_config_args(m20_dan_nav_sim.config(), (), override_path)
    atom = next(
        atom for atom in m20_dan_nav_sim.blueprints if atom.module is M20MujocoSimConnection
    )
    merged = {**atom.kwargs, **overrides["m20mujocosimconnection"]}
    config = M20MujocoSimConfig.model_validate(merged)

    assert config.pointcloud_fps == 1.0
    assert config.enable_color
    assert config.publish_front_image
