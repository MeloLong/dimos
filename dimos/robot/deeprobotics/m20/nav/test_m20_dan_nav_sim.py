from dimos.robot.deeprobotics.m20.connection import M20Connection
from dimos.robot.deeprobotics.m20.mujoco_sim import M20MujocoSimConnection
from dimos.robot.deeprobotics.m20.nav.m20_dan_nav import m20_dan_nav, m20_dan_nav_sim


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
