#!/usr/bin/env python3
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

"""Python NativeModule wrappers for M20 C++ sensor drivers.

Each hardware driver has its own native executable. The combined
``m20-native-sensors`` blueprint starts both modules and exposes them as one
sensor stack.
"""

from __future__ import annotations

from pydantic import Field

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import Out
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.spec import perception


class M20LidarConfig(NativeModuleConfig):
    cwd: str | None = "cpp/lidar"
    executable: str = "result/bin/m20_lidar_native"
    build_command: str | None = "cmake -B build && cmake --build build --target install -j"

    ip: str = Field(default_factory=lambda m: m["g"].robot_ip or "192.168.1.20")
    lidar_hz: float = 20.0
    lidar_frame_id: str = "lidar"

    # todo: Add M20 lidar SDK-specific config fields here, such as ports,
    # device IDs, calibration paths, return mode, or packet format.


class M20CameraConfig(NativeModuleConfig):
    cwd: str | None = "cpp/camera"
    executable: str = "result/bin/m20_camera_native"
    build_command: str | None = "cmake -B build && cmake --build build --target install -j"

    ip: str = Field(default_factory=lambda m: m["g"].robot_ip or "192.168.1.20")
    camera_hz: float = 30.0
    camera_info_hz: float = 1.0
    camera_frame_id: str = "camera_optical"

    # todo: Add M20 camera SDK-specific config fields here, such as device ID,
    # calibration path, exposure mode, pixel format, or stream profile.


class M20Lidar(NativeModule, perception.Lidar):
    """Expose the M20 native lidar driver as a DimOS point cloud stream."""

    config: M20LidarConfig

    lidar: Out[PointCloud2]


class M20Camera(NativeModule, perception.Camera):
    """Expose the M20 native camera driver as DimOS image streams."""

    config: M20CameraConfig

    color_image: Out[Image]
    camera_info: Out[CameraInfo]


# Backwards-compatible alias for local lidar-only experiments.
# M20Sensor = M20Lidar
