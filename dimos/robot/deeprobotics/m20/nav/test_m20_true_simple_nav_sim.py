import yaml

from dimos.mapping.costmapper import CostMapper
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos.robot.deeprobotics.m20.connection import M20Connection
from dimos.robot.deeprobotics.m20.mujoco_sim import M20MujocoSimConfig, M20MujocoSimConnection
from dimos.robot.deeprobotics.m20.nav.m20_true_simple_nav import (
    M20_MUJOCO_SIM_CONFIG_PATH,
    m20_true_simple_nav,
    m20_true_simple_nav_sim,
)


def _modules(blueprint):
    return {atom.module for atom in blueprint.blueprints}


def test_true_simple_nav_real_and_sim_connections_are_isolated() -> None:
    real_modules = _modules(m20_true_simple_nav)
    sim_modules = _modules(m20_true_simple_nav_sim)

    assert M20Connection in real_modules
    assert M20MujocoSimConnection not in real_modules
    assert M20MujocoSimConnection in sim_modules
    assert M20Connection not in sim_modules


def test_true_simple_nav_sim_feeds_m20_slam_topics() -> None:
    remappings = m20_true_simple_nav_sim.remapping_map

    assert remappings[(M20MujocoSimConnection, "slam_odom")] == "dimos/slam_odom"
    assert (
        remappings[(M20MujocoSimConnection, "slam_aligned_points")] == "dimos/slam_aligned_points"
    )


def test_true_simple_nav_sim_uses_go1_envelope_from_yaml() -> None:
    payload = yaml.safe_load(M20_MUJOCO_SIM_CONFIG_PATH.read_text(encoding="utf-8"))
    clearance = payload["mlsplannernative"]["wall_clearance_m"]
    height = payload["mlsplannernative"]["robot_height"]

    cost_mapper = next(
        atom for atom in m20_true_simple_nav_sim.blueprints if atom.module is CostMapper
    )
    planner = next(
        atom for atom in m20_true_simple_nav_sim.blueprints if atom.module is ReplanningAStarPlanner
    )

    assert cost_mapper.kwargs["config"].can_pass_under == height
    assert cost_mapper.kwargs["initial_safe_radius_meters"] == clearance
    assert planner.kwargs["robot_width"] == clearance * 2
    assert planner.kwargs["robot_rotation_diameter"] == clearance * 2
    assert m20_true_simple_nav_sim.global_config_overrides["robot_model"] == "unitree_go1"


def test_true_simple_nav_sim_loads_checked_in_sensor_profile() -> None:
    payload = yaml.safe_load(M20_MUJOCO_SIM_CONFIG_PATH.read_text(encoding="utf-8"))
    values = payload["m20mujocosimconnection"]
    expected = M20MujocoSimConfig.model_validate(values).model_dump(include=set(values))
    simulator = next(
        atom for atom in m20_true_simple_nav_sim.blueprints if atom.module is M20MujocoSimConnection
    )

    assert simulator.kwargs == expected
