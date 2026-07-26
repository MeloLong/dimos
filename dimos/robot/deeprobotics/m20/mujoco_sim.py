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

"""MuJoCo-backed sensor/control adapter for testing the M20 navigation stack.

The shared DimOS MuJoCo process loads the vendored official M20 MJCF and ONNX
policy. This adapter publishes the streams expected by the M20 Simple Nav and
DAN blueprints and keeps simulation-only command tuning out of the real-robot
connection.
"""

from __future__ import annotations

from typing import Any

from pydantic import Field, model_validator
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.unitree.mujoco_connection import MujocoConnection
from dimos.robot.unitree.type.odometry import Odometry as SimOdometry
from dimos.simulation.mujoco.sensor_config import MujocoSensorConfig


class M20MujocoSimConfig(ModuleConfig, MujocoSensorConfig):
    """M20 topic publication plus legacy MuJoCo sensor compute settings."""

    publish_front_image: bool = True
    publish_rear_image: bool = False
    person_collision_enabled: bool = False
    yaw_command_scale: float = Field(default=1.0, gt=0.0)
    yaw_command_limit: float = Field(default=1.6, gt=0.0)

    @model_validator(mode="after")
    def validate_image_publication(self) -> M20MujocoSimConfig:
        if not self.enable_color and (self.publish_front_image or self.publish_rear_image):
            raise ValueError("image publication requires enable_color=True")
        return self

    def sensor_config(self) -> MujocoSensorConfig:
        fields = set(MujocoSensorConfig.model_fields)
        return MujocoSensorConfig.model_validate(self.model_dump(include=fields))


def _adapt_yaw_command(twist: Twist, scale: float, limit: float) -> Twist:
    adjusted = Twist(twist)
    adjusted.angular.z = max(-limit, min(limit, twist.angular.z * scale))
    return adjusted


class M20MujocoSimConnection(Module):
    """Publish MuJoCo sim data on the M20 nav topics."""

    dedicated_worker = True

    config: M20MujocoSimConfig
    cmd_vel: In[Twist]
    slam_aligned_points: Out[PointCloud2]
    slam_odom: Out[Odometry]
    color_image: Out[Image]
    color_image_rear: Out[Image]

    connection: MujocoConnection | None = None

    @rpc
    def start(self) -> None:
        super().start()

        # Keep the DimOS/Rerun viewer available while forcing MuJoCo itself to
        # run as a background data source without opening its own window.
        sim_config = self.config.g.model_copy(
            update={
                "viewer": "none",
                "mujoco_person_collision_enabled": self.config.person_collision_enabled,
            }
        )
        self.connection = MujocoConnection(sim_config, self.config.sensor_config())
        self.connection.start()

        self.register_disposable(Disposable(self.cmd_vel.subscribe(self.move)))
        self.register_disposable(self.connection.odom_stream().subscribe(self._publish_odom))
        if self.config.enable_pointcloud:
            self.register_disposable(
                self.connection.lidar_stream().subscribe(self.slam_aligned_points.publish)
            )
        if self.config.enable_color:
            self.register_disposable(self.connection.video_stream().subscribe(self._publish_video))

    @rpc
    def stop(self) -> None:
        if self.connection is not None:
            self.connection.stop()
            self.connection = None
        super().stop()

    def _publish_odom(self, msg: SimOdometry) -> None:
        self.slam_odom.publish(
            Odometry(
                ts=msg.ts,
                frame_id="map",
                child_frame_id="base_link",
                pose=Pose(msg.position, msg.orientation),
            )
        )

    def _publish_video(self, image: Image) -> None:
        if self.config.publish_front_image:
            self.color_image.publish(image)
        if self.config.publish_rear_image:
            self.color_image_rear.publish(image)

    @rpc
    def move(self, twist: Twist, duration: float = 0.0) -> bool:
        if self.connection is None:
            return True
        adjusted = _adapt_yaw_command(
            twist,
            self.config.yaw_command_scale,
            self.config.yaw_command_limit,
        )
        return self.connection.move(adjusted, duration)

    @rpc
    def publish_request(self, topic: str, data: dict[str, Any]) -> dict[Any, Any]:
        if self.connection is None:
            return {}
        return self.connection.publish_request(topic, data)
