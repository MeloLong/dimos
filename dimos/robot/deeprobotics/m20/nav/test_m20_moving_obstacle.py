import math
from unittest.mock import MagicMock, patch

import mujoco
from pydantic import ValidationError
import pytest

from dimos.robot.deeprobotics.m20.nav.moving_obstacle import (
    M20MovingObstacle,
    M20MovingObstacleConfig,
    RandomWaypointWalk,
)
from dimos.simulation.mujoco.model import get_assets, get_model_xml
from dimos.simulation.mujoco.person_on_track import PersonPositionController
from dimos.utils.data import get_data


def _config(**overrides: object) -> M20MovingObstacleConfig:
    return M20MovingObstacleConfig.model_validate(
        {
            "waypoints": [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)],
            **overrides,
        }
    )


def test_random_walk_is_reproducible() -> None:
    first = RandomWaypointWalk(_config(seed=7, speed_mps=1.0))
    second = RandomWaypointWalk(_config(seed=7, speed_mps=1.0))

    first_positions = [first.step(1.1).position for _ in range(8)]
    second_positions = [second.step(1.1).position for _ in range(8)]

    assert first_positions == second_positions


def test_random_walk_respects_speed() -> None:
    walker = RandomWaypointWalk(_config(seed=1, speed_mps=0.2))
    start = walker.position

    walker.step(0.5)

    assert math.dist(start, walker.position) == pytest.approx(0.1)


def test_random_walk_pose_has_unit_quaternion() -> None:
    pose = RandomWaypointWalk(_config()).pose()

    assert math.sqrt(sum(value * value for value in pose.orientation.to_tuple())) == pytest.approx(
        1.0
    )


def test_moving_obstacle_starts_timer_and_releases_transport() -> None:
    config = _config(update_hz=20.0)
    with patch(
        "dimos.robot.deeprobotics.m20.nav.moving_obstacle.Module.__init__",
        autospec=True,
    ) as module_init:
        module_init.return_value = None
        obstacle = M20MovingObstacle(**config.model_dump())

    module_init.assert_called_once()
    obstacle.config = config
    obstacle.register_disposable = MagicMock()
    transport = MagicMock()
    interval = MagicMock()
    disposable = MagicMock()
    interval.subscribe.return_value = disposable

    with (
        patch("dimos.robot.deeprobotics.m20.nav.moving_obstacle.Module.start"),
        patch("dimos.robot.deeprobotics.m20.nav.moving_obstacle.Module.stop"),
        patch(
            "dimos.robot.deeprobotics.m20.nav.moving_obstacle.make_transport",
            return_value=transport,
        ),
        patch(
            "dimos.robot.deeprobotics.m20.nav.moving_obstacle.rx.interval",
            return_value=interval,
        ) as make_interval,
    ):
        obstacle.start()
        on_next = interval.subscribe.call_args.kwargs["on_next"]
        on_next(0)
        obstacle.stop()

    make_interval.assert_called_once_with(0.05)
    obstacle.register_disposable.assert_called_once_with(disposable)
    assert transport.broadcast.call_count == 2
    transport.stop.assert_called_once_with()


@pytest.mark.mujoco
def test_random_walk_pose_drives_noncolliding_pointcloud_person() -> None:
    scene_path = get_data("mujoco_sim") / "scene_office1.xml"
    xml = get_model_xml(
        "unitree_go1",
        scene_path.read_text(encoding="utf-8"),
        person_collision_enabled=False,
    )
    model = mujoco.MjModel.from_xml_string(xml, assets=get_assets())
    data = mujoco.MjData(model)
    person_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "person")
    person_mocap_id = int(model.body_mocapid[person_body_id])
    person_geom_ids = [
        geom_id
        for geom_id in range(model.ngeom)
        if int(model.geom_bodyid[geom_id]) == person_body_id
    ]

    controller = PersonPositionController.__new__(PersonPositionController)
    controller._person_mocap_id = person_mocap_id
    controller._latest_pose = RandomWaypointWalk(_config()).step(0.5)
    controller.tick(data)

    pose = controller._latest_pose
    assert pose is not None
    assert data.mocap_pos[person_mocap_id].tolist() == pytest.approx(
        [pose.position.x, pose.position.y, pose.position.z]
    )
    assert person_geom_ids
    assert all(int(model.geom_group[geom_id]) == 0 for geom_id in person_geom_ids)
    assert all(int(model.geom_contype[geom_id]) == 0 for geom_id in person_geom_ids)
    assert all(int(model.geom_conaffinity[geom_id]) == 0 for geom_id in person_geom_ids)


@pytest.mark.parametrize(
    "overrides",
    [
        {"waypoints": [(0.0, 0.0)]},
        {"waypoints": [(0.0, 0.0), (0.0, 0.0)]},
        {"initial_waypoint_index": 4},
        {"waypoints": [(0.0, 0.0), (math.inf, 1.0)]},
    ],
)
def test_moving_obstacle_config_rejects_invalid_waypoints(
    overrides: dict[str, object],
) -> None:
    with pytest.raises(ValidationError):
        _config(**overrides)
