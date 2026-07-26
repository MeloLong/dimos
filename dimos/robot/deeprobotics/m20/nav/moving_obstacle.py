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

"""Deterministic moving-person obstacle for the M20 MuJoCo nav blueprint."""

from __future__ import annotations

import math
import random
from typing import Literal

from pydantic import Field, model_validator
import reactivex as rx
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.core.transport import PubSubTransport
from dimos.core.transport_factory import make_transport
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

PERSON_POSE_TOPIC = "/person_pose"


class M20MovingObstacleConfig(ModuleConfig):
    """A single person moving along validated office-scene path edges."""

    enabled: bool = True
    seed: int = 20
    speed_mps: float = Field(default=0.15, gt=0.0, le=1.0)
    update_hz: float = Field(default=20.0, gt=0.0, le=60.0)
    initial_waypoint_index: int = Field(default=0, ge=0)
    z_m: float = 0.0
    waypoints: list[tuple[float, float]] = Field(
        default_factory=lambda: [
            (-3.35, -0.51),
            (-2.60, 1.28),
            (4.80, 0.21),
            (4.14, -6.00),
            (0.59, -3.79),
        ]
    )
    proximity_stop_distance_m: float = Field(default=0.9, gt=0.0, le=10.0)
    proximity_resume_distance_m: float = Field(default=1.1, gt=0.0, le=10.0)
    proximity_pause_s: float = Field(default=1.0, ge=0.0, le=60.0)

    @model_validator(mode="after")
    def validate_waypoints(self) -> M20MovingObstacleConfig:
        if len(self.waypoints) < 2:
            raise ValueError("moving obstacle requires at least two waypoints")
        if len(set(self.waypoints)) != len(self.waypoints):
            raise ValueError("moving obstacle waypoints must be unique")
        if self.initial_waypoint_index >= len(self.waypoints):
            raise ValueError("initial_waypoint_index is outside the waypoint list")
        if any(not math.isfinite(value) for point in self.waypoints for value in point):
            raise ValueError("moving obstacle waypoints must be finite")
        if self.proximity_resume_distance_m <= self.proximity_stop_distance_m:
            raise ValueError(
                "proximity_resume_distance_m must be greater than proximity_stop_distance_m"
            )
        return self


class RandomWaypointWalk:
    """Move at constant speed while randomly choosing either adjacent path edge."""

    def __init__(self, config: M20MovingObstacleConfig) -> None:
        self._waypoints = tuple(config.waypoints)
        self._speed_mps = config.speed_mps
        self._z_m = config.z_m
        self._rng = random.Random(config.seed)
        self._current_waypoint_index = config.initial_waypoint_index
        self._position = list(self._waypoints[self._current_waypoint_index])
        self._target_waypoint_index = self._choose_target()

    @property
    def position(self) -> tuple[float, float]:
        return (self._position[0], self._position[1])

    @property
    def target_waypoint_index(self) -> int:
        return self._target_waypoint_index

    @property
    def target_position(self) -> tuple[float, float]:
        return self._waypoints[self._target_waypoint_index]

    @property
    def speed_mps(self) -> float:
        return self._speed_mps

    @property
    def at_waypoint(self) -> bool:
        return self.position == self._waypoints[self._current_waypoint_index]

    def is_moving_toward(self, point: tuple[float, float]) -> bool:
        target = self.target_position
        travel_x = target[0] - self._position[0]
        travel_y = target[1] - self._position[1]
        point_x = point[0] - self._position[0]
        point_y = point[1] - self._position[1]
        return travel_x * point_x + travel_y * point_y > 0.0

    def redirect_away_from(self, point: tuple[float, float]) -> bool:
        """Choose the legal adjacent direction with the largest separation gain."""
        current = self._current_waypoint_index
        target = self._target_waypoint_index
        if not self.at_waypoint:
            candidates = (current, target)
        else:
            count = len(self._waypoints)
            candidates = ((current - 1) % count, (current + 1) % count)

        away_x = self._position[0] - point[0]
        away_y = self._position[1] - point[1]
        selected = max(
            candidates,
            key=lambda index: (
                (self._waypoints[index][0] - self._position[0]) * away_x
                + (self._waypoints[index][1] - self._position[1]) * away_y
            ),
        )
        separation_gain = (self._waypoints[selected][0] - self._position[0]) * away_x + (
            self._waypoints[selected][1] - self._position[1]
        ) * away_y
        if separation_gain <= 0.0:
            return False
        if not self.at_waypoint:
            # The non-target endpoint becomes the logical edge origin.
            self._current_waypoint_index = target
        self._target_waypoint_index = selected
        return True

    def step(self, dt_seconds: float, *, stop_at_waypoint: bool = False) -> Pose:
        if not math.isfinite(dt_seconds) or dt_seconds < 0.0:
            raise ValueError("dt_seconds must be finite and non-negative")

        remaining = self._speed_mps * dt_seconds
        while remaining > 0.0:
            target = self._waypoints[self._target_waypoint_index]
            dx = target[0] - self._position[0]
            dy = target[1] - self._position[1]
            distance = math.hypot(dx, dy)

            travel = min(remaining, distance)
            self._position[0] += dx / distance * travel
            self._position[1] += dy / distance * travel
            remaining -= travel

            if travel == distance:
                self._arrive_at_target()
                if stop_at_waypoint:
                    break

        return self.pose()

    def pose(self) -> Pose:
        target = self._waypoints[self._target_waypoint_index]
        heading = math.atan2(target[1] - self._position[1], target[0] - self._position[0])
        # The person mesh faces the opposite direction of the path frame.
        yaw = heading + math.pi
        return Pose(
            position=[self._position[0], self._position[1], self._z_m],
            orientation=[0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)],
        )

    def _arrive_at_target(self) -> None:
        self._current_waypoint_index = self._target_waypoint_index
        self._position = list(self._waypoints[self._current_waypoint_index])
        self._target_waypoint_index = self._choose_target()

    def _choose_target(self) -> int:
        count = len(self._waypoints)
        candidates = sorted(
            {
                (self._current_waypoint_index - 1) % count,
                (self._current_waypoint_index + 1) % count,
            }
        )
        return self._rng.choice(candidates)


class M20MovingObstacle(Module):
    """Publish one deterministic pseudo-random person pose into MuJoCo."""

    config: M20MovingObstacleConfig
    odometry: In[Odometry]
    _transport: PubSubTransport[Pose] | None = None
    _walker: RandomWaypointWalk | None = None
    _robot_position: tuple[float, float] | None = None
    _avoidance_state: Literal["normal", "paused", "retreat"] = "normal"
    _pause_remaining_s: float = 0.0

    @rpc
    def start(self) -> None:
        super().start()
        if not self.config.enabled:
            return

        self._transport = make_transport(PERSON_POSE_TOPIC, Pose)
        self._walker = RandomWaypointWalk(self.config)
        self._avoidance_state = "normal"
        self._pause_remaining_s = 0.0
        self.register_disposable(Disposable(self.odometry.subscribe(self._on_odometry)))
        self._publish_pose(self._walker.pose())
        self.register_disposable(
            rx.interval(1.0 / self.config.update_hz).subscribe(
                on_next=self._tick,
                on_error=lambda error: logger.error(
                    "M20 moving obstacle update failed", error=str(error)
                ),
            )
        )

    @rpc
    def stop(self) -> None:
        transport = self._transport
        self._transport = None
        self._walker = None
        self._robot_position = None
        self._avoidance_state = "normal"
        self._pause_remaining_s = 0.0
        try:
            super().stop()
        finally:
            if transport is not None:
                transport.stop()

    def _tick(self, _index: int) -> None:
        if self._walker is None:
            return
        self._publish_pose(self._next_pose(1.0 / self.config.update_hz))

    def _on_odometry(self, odometry: Odometry) -> None:
        self._robot_position = (odometry.position.x, odometry.position.y)

    def _next_pose(self, dt_seconds: float) -> Pose:
        walker = self._walker
        robot = self._robot_position
        if walker is None or robot is None:
            return walker.step(dt_seconds) if walker is not None else Pose()

        distance = math.dist(walker.position, robot)
        if self._avoidance_state == "normal":
            stopping_margin = walker.speed_mps * dt_seconds
            if distance <= self.config.proximity_stop_distance_m + stopping_margin:
                walker.redirect_away_from(robot)
                self._avoidance_state = "paused"
                self._pause_remaining_s = self.config.proximity_pause_s
                logger.info(
                    "M20 moving obstacle stopping near robot",
                    distance_m=round(distance, 3),
                )
                if self._pause_remaining_s > 0.0:
                    return walker.pose()

        if self._avoidance_state == "paused":
            self._pause_remaining_s = max(0.0, self._pause_remaining_s - dt_seconds)
            if self._pause_remaining_s > 0.0:
                return walker.pose()
            self._avoidance_state = "retreat"
            logger.info("M20 moving obstacle redirecting away from robot")

        if self._avoidance_state == "retreat":
            if distance >= self.config.proximity_resume_distance_m:
                self._avoidance_state = "normal"
                logger.info(
                    "M20 moving obstacle resuming random walk",
                    distance_m=round(distance, 3),
                )
            elif walker.at_waypoint and not walker.redirect_away_from(robot):
                return walker.pose()
            elif (
                distance <= self.config.proximity_stop_distance_m + walker.speed_mps * dt_seconds
                and walker.is_moving_toward(robot)
            ):
                walker.redirect_away_from(robot)

        return walker.step(
            dt_seconds,
            stop_at_waypoint=self._avoidance_state == "retreat",
        )

    def _publish_pose(self, pose: Pose) -> None:
        if self._transport is not None:
            self._transport.broadcast(None, pose)
