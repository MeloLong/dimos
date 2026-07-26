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

from typing import Any

import numpy as np
from reactivex import empty

from dimos.robot.unitree.mujoco_connection import MujocoConnection
from dimos.simulation.mujoco.sensor_config import MujocoSensorConfig


def test_video_stream_uses_configured_frame_id() -> None:
    connection = object.__new__(MujocoConnection)
    connection.sensor_config = MujocoSensorConfig(color_frame_id="test_camera_optical")
    connection.get_video_frame = lambda: np.zeros((2, 3, 3), dtype=np.uint8)
    captured: dict[str, Any] = {}

    def capture_frame(getter: Any, _frequency: float, _stream_name: str) -> Any:
        captured["image"] = getter()
        return empty()

    connection._create_stream = capture_frame
    try:
        connection.video_stream()
    finally:
        MujocoConnection.video_stream.cache_clear()

    assert captured["image"].frame_id == "test_camera_optical"
