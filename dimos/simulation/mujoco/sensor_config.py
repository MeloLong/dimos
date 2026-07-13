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

"""Validated sensor settings for the legacy Unitree MuJoCo connection."""

from typing import Annotated

from pydantic import Field

from dimos.protocol.service.spec import BaseConfig
from dimos.simulation.mujoco.constants import (
    DEPTH_CAMERA_FOV,
    LIDAR_FPS,
    LIDAR_RESOLUTION,
    VIDEO_CAMERA_FOV,
    VIDEO_FPS,
    VIDEO_HEIGHT,
    VIDEO_WIDTH,
)


class MujocoSensorConfig(BaseConfig):
    """Sensor compute settings shared by the parent and MuJoCo subprocess."""

    enable_color: bool = True
    color_camera_name: str = Field(default="head_camera", min_length=1)
    color_frame_id: str = Field(default="camera_optical", min_length=1)
    width: int = Field(default=VIDEO_WIDTH, gt=0)
    height: int = Field(default=VIDEO_HEIGHT, gt=0)
    fps: float = Field(default=VIDEO_FPS, gt=0)
    color_fov_deg: float = Field(default=VIDEO_CAMERA_FOV, gt=0, lt=180)

    enable_pointcloud: bool = True
    pointcloud_fps: float = Field(default=LIDAR_FPS, gt=0)
    pointcloud_camera_names: tuple[str, ...] = Field(
        default=("lidar_front_camera", "lidar_left_camera", "lidar_right_camera"),
        min_length=1,
    )
    pointcloud_fov_deg: float = Field(default=DEPTH_CAMERA_FOV, gt=0, lt=180)
    pointcloud_voxel_size: float = Field(default=LIDAR_RESOLUTION, gt=0)
    pointcloud_geom_groups: tuple[
        Annotated[int, Field(ge=0, le=5)], ...
    ] = Field(default=(0, 1, 2), min_length=1)
