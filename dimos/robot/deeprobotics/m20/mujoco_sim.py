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

This reuses the existing quadruped MuJoCo simulator as a navigation data source
until a real M20 MuJoCo model is available. It publishes M20-shaped topics so
the `m20-dan-nav` chain can be exercised without the robot.
"""

from __future__ import annotations

from typing import Any

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


class M20MujocoSimConfig(ModuleConfig):
    pass


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
        sim_config = self.config.g.model_copy(update={"viewer": "none"})
        self.connection = MujocoConnection(sim_config)
        self.connection.start()

        self.register_disposable(Disposable(self.cmd_vel.subscribe(self.move)))
        self.register_disposable(self.connection.odom_stream().subscribe(self._publish_odom))
        self.register_disposable(
            self.connection.lidar_stream().subscribe(self.slam_aligned_points.publish)
        )
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
        self.color_image.publish(image)
        self.color_image_rear.publish(image)

    @rpc
    def move(self, twist: Twist, duration: float = 0.0) -> bool:
        if self.connection is None:
            return True
        return self.connection.move(twist, duration)

    @rpc
    def publish_request(self, topic: str, data: dict[str, Any]) -> dict[Any, Any]:
        if self.connection is None:
            return {}
        return self.connection.publish_request(topic, data)
