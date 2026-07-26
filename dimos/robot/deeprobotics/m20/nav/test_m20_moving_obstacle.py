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

import math
from unittest.mock import MagicMock, patch

import mujoco
from pydantic import ValidationError
import pytest

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.nav_msgs.Odometry import Odometry
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


def test_enabled_override_uses_default_waypoints() -> None:
    config = M20MovingObstacleConfig(enabled=True)

    assert config.enabled is True
    assert len(config.waypoints) == 5


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


def test_random_walk_redirect_stays_on_current_edge() -> None:
    walker = RandomWaypointWalk(_config(seed=1, speed_mps=1.0))
    initial_target = walker.target_position
    walker.step(0.25)

    assert walker.redirect_away_from(initial_target) is True
    redirected_target = walker.target_position

    assert redirected_target == (0.0, 0.0)
    assert walker.position[0] == pytest.approx(0.25 * initial_target[0])
    assert walker.position[1] == pytest.approx(0.25 * initial_target[1])


def test_random_walk_holds_when_no_adjacent_edge_increases_separation() -> None:
    walker = RandomWaypointWalk(_config(initial_waypoint_index=0))

    redirected = walker.redirect_away_from((0.5, 0.5))

    assert redirected is False
    assert walker.position == (0.0, 0.0)


def _avoidance_obstacle(config: M20MovingObstacleConfig) -> M20MovingObstacle:
    with patch(
        "dimos.robot.deeprobotics.m20.nav.moving_obstacle.Module.__init__",
        autospec=True,
    ) as module_init:
        module_init.return_value = None
        obstacle = M20MovingObstacle(**config.model_dump())
    obstacle.config = config
    obstacle._walker = RandomWaypointWalk(config)
    obstacle._avoidance_state = "normal"
    obstacle._pause_remaining_s = 0.0
    return obstacle


def test_proximity_avoidance_pauses_then_moves_away() -> None:
    config = _config(
        seed=1,
        speed_mps=0.2,
        update_hz=10.0,
        proximity_stop_distance_m=0.5,
        proximity_resume_distance_m=0.8,
        proximity_pause_s=0.2,
    )
    obstacle = _avoidance_obstacle(config)
    assert obstacle._walker is not None
    target = obstacle._walker.target_position
    obstacle._walker.step(1.0)
    obstacle._robot_position = (target[0] * 0.5, target[1] * 0.5)
    start = obstacle._walker.position

    first = obstacle._next_pose(0.1)
    second = obstacle._next_pose(0.1)
    third = obstacle._next_pose(0.1)

    assert first.position.to_tuple()[:2] == pytest.approx(start)
    assert second.position.to_tuple()[:2] == pytest.approx(start)
    assert obstacle._avoidance_state == "retreat"
    assert math.dist(third.position.to_tuple()[:2], obstacle._robot_position) > math.dist(
        start, obstacle._robot_position
    )


def test_proximity_avoidance_uses_resume_hysteresis() -> None:
    config = _config(
        seed=1,
        speed_mps=0.2,
        update_hz=10.0,
        proximity_stop_distance_m=0.5,
        proximity_resume_distance_m=0.8,
        proximity_pause_s=0.0,
    )
    obstacle = _avoidance_obstacle(config)
    assert obstacle._walker is not None
    target = obstacle._walker.target_position
    obstacle._walker.step(1.0)
    obstacle._robot_position = (target[0] * 0.5, target[1] * 0.5)

    obstacle._next_pose(0.1)
    assert obstacle._avoidance_state == "retreat"

    for _ in range(8):
        previous_distance = math.dist(obstacle._walker.position, obstacle._robot_position)
        obstacle._next_pose(0.1)
        current_distance = math.dist(obstacle._walker.position, obstacle._robot_position)
        assert current_distance > previous_distance
        assert obstacle._avoidance_state == "retreat"


def test_proximity_stop_also_triggers_when_robot_approaches_person() -> None:
    config = _config(
        seed=1,
        speed_mps=0.2,
        proximity_stop_distance_m=0.5,
        proximity_resume_distance_m=0.8,
        proximity_pause_s=0.2,
    )
    obstacle = _avoidance_obstacle(config)
    assert obstacle._walker is not None
    obstacle._walker.step(1.0)
    obstacle._robot_position = (0.0, 0.0)
    start = obstacle._walker.position

    pose = obstacle._next_pose(0.1)

    assert pose.position.to_tuple()[:2] == pytest.approx(start)
    assert obstacle._avoidance_state == "paused"


def test_moving_obstacle_tracks_robot_odometry() -> None:
    obstacle = _avoidance_obstacle(_config())

    obstacle._on_odometry(Odometry(pose=Pose(position=[1.25, -0.75, 0.0])))

    assert obstacle._robot_position == (1.25, -0.75)


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
    obstacle.odometry = MagicMock()
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
    assert obstacle.register_disposable.call_count == 2
    obstacle.register_disposable.assert_any_call(disposable)
    obstacle.odometry.subscribe.assert_called_once_with(obstacle._on_odometry)
    assert transport.broadcast.call_count == 2
    transport.stop.assert_called_once_with()


@pytest.mark.mujoco
def test_random_walk_pose_drives_noncolliding_pointcloud_person() -> None:
    scene_path = get_data("mujoco_sim") / "scene_office1.xml"
    xml = get_model_xml(
        "deeprobotics_m20",
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
        {"proximity_stop_distance_m": 1.0, "proximity_resume_distance_m": 1.0},
    ],
)
def test_moving_obstacle_config_rejects_invalid_waypoints(
    overrides: dict[str, object],
) -> None:
    with pytest.raises(ValidationError):
        _config(**overrides)
