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

"""M20 map accumulation and save stack.

This blueprint follows the M20 startup, SLAM-topic subscriptions, and Rerun
viewer setup used by ``m20_dan_nav``, but omits navigation. It keyframe-
accumulates ``slam_aligned_points`` with ``slam_odom`` and saves the map when
the run shuts down gracefully.
"""

import math
from pathlib import Path

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.robot.deeprobotics.m20.blueprints.basic import m20_rerun_blueprint
from dimos.robot.deeprobotics.m20.connection import M20Connection
from dimos.robot.deeprobotics.m20.nav.map_save.map_save import PointCloudMapSave
from dimos.robot.deeprobotics.m20.tf import M20TF
from dimos.visualization.vis_module import vis_module

voxel_size = 0.05
map_save_dir = Path(__file__).resolve().parent / "map_save"
map_save_path = map_save_dir / "m20_accumulated_map.pcd"

_m20_map_save_rerun_config = {
    "blueprint": m20_rerun_blueprint,
    "memory_limit": "1GB",
    "max_hz": {
        "world/color_image": 0,
        "world/color_image_rear": 0,
        "world/global_map": 1.0,
    },
    "latest_only_entities": [
        "world/slam_aligned_points",
        "world/global_map",
    ],
}


_m20_map_save_base = autoconnect(
    vis_module(viewer_backend=global_config.viewer, rerun_config=_m20_map_save_rerun_config),
    M20Connection.blueprint(),
    M20TF.blueprint().remappings([(M20TF, "odometry", "dimos/slam_odom")]),
)


m20_map_save = autoconnect(
    _m20_map_save_base,
    PointCloudMapSave.blueprint(
        translation_threshold_m=0.5,
        rotation_threshold_rad=math.radians(15.0),
        voxel_size=voxel_size,
        world_frame_id="map",
        save_path=str(map_save_path),
    ).remappings(
        [
            (PointCloudMapSave, "lidar", "dimos/slam_aligned_points"),
            (PointCloudMapSave, "odometry", "dimos/slam_odom"),
        ]
    ),
    MovementManager.blueprint(),
).global_config(n_workers=5, robot_model="m20")

__all__ = ["m20_map_save"]
