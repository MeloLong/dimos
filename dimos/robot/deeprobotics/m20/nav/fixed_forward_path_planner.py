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

"""Fixed forward path planner for M20 bringup/debugging.

This module is intentionally much simpler than MLSPlannerNative: when a goal is
received, it ignores the goal position and tries to publish a straight path 4 m
along the robot's current +X axis. The path is only published when the latest
local map has no occupied voxels inside the configured forward corridor.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

import numpy as np
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.LineSegments3D import LineSegments3D
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class FixedForwardPathPlannerConfig(ModuleConfig):
    path_length_m: float = 4.0
    safety_extension_m: float = 0.3
    sample_spacing_m: float = 0.2
    corridor_radius_m: float = 0.6
    min_relative_z_m: float = -0.2
    max_relative_z_m: float = 1.2
    min_blocking_points: int = 3


@dataclass(frozen=True)
class BlockedReason:
    count: int
    nearest_forward_m: float
    nearest_lateral_m: float
    nearest_relative_z_m: float

    def format(self) -> str:
        return (
            f"{self.count} occupied point(s) in forward corridor; "
            f"nearest at x={self.nearest_forward_m:.2f}m, "
            f"lateral={self.nearest_lateral_m:.2f}m, "
            f"relative_z={self.nearest_relative_z_m:.2f}m"
        )


class FixedForwardPathPlanner(Module):
    """Publish a straight 4 m path along the robot's current +X axis on goal.

    The ports mirror MLSPlannerNative's public contract so this module can be
    swapped into M20 navigation blueprints with minimal remapping changes.
    """

    config: FixedForwardPathPlannerConfig

    global_map: In[PointCloud2]
    local_map: In[PointCloud2]
    region_bounds: In[PoseStamped]
    start_pose: In[PoseStamped]
    goal_pose: In[PoseStamped]

    path: Out[Path]
    surface_map: Out[PointCloud2]
    nodes: Out[PointCloud2]
    node_edges: Out[LineSegments3D]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._latest_local_map: PointCloud2 | None = None
        self._latest_start_pose: PoseStamped | None = None
        self._latest_region_bounds: PoseStamped | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.local_map.subscribe(self._on_local_map)))
        self.register_disposable(Disposable(self.start_pose.subscribe(self._on_start_pose)))
        self.register_disposable(Disposable(self.region_bounds.subscribe(self._on_region_bounds)))
        self.register_disposable(Disposable(self.goal_pose.subscribe(self._on_goal_pose)))

    def _on_local_map(self, msg: PointCloud2) -> None:
        self._latest_local_map = msg

    def _on_start_pose(self, msg: PoseStamped) -> None:
        self._latest_start_pose = msg

    def _on_region_bounds(self, msg: PoseStamped) -> None:
        self._latest_region_bounds = msg

    def _on_goal_pose(self, goal: PoseStamped) -> None:
        if not self._finite_pose(goal):
            self._fail("received non-finite goal_pose")
            return

        start = self._latest_start_pose
        if start is None:
            self._fail("missing start_pose")
            return
        if not self._finite_pose(start):
            self._fail("latest start_pose is non-finite")
            return

        local_map = self._latest_local_map
        if local_map is None:
            self._fail("missing local_map")
            return

        blocked = self._blocked_reason(start, local_map)
        if blocked is not None:
            self._fail(f"blocked: {blocked.format()}")
            return

        path = self._make_path(start)
        self.path.publish(path)
        logger.info(
            "FixedForwardPathPlanner published path",
            frame_id=path.frame_id,
            poses=len(path.poses),
            length_m=self.config.path_length_m,
        )

    def _make_path(self, start: PoseStamped) -> Path:
        spacing = max(self.config.sample_spacing_m, 1e-3)
        steps = max(1, math.ceil(self.config.path_length_m / spacing))
        poses: list[PoseStamped] = []
        for i in range(steps + 1):
            distance = min(i * spacing, self.config.path_length_m)
            offset = start.orientation.rotate_vector(Vector3(distance, 0.0, 0.0))
            poses.append(
                PoseStamped(
                    ts=start.ts,
                    frame_id=start.frame_id,
                    position=[
                        start.x + offset.x,
                        start.y + offset.y,
                        start.z + offset.z,
                    ],
                    orientation=start.orientation,
                )
            )
        return Path(frame_id=start.frame_id, poses=poses)

    def _blocked_reason(self, start: PoseStamped, local_map: PointCloud2) -> BlockedReason | None:
        points = local_map.points_f32()
        if points.size == 0:
            return None

        origin = np.array([start.x, start.y, start.z], dtype=np.float32)
        relative_world = points[:, :3] - origin
        rotation_inv = start.orientation.inverse().to_rotation_matrix().astype(np.float32)
        relative_robot = relative_world @ rotation_inv.T

        forward = relative_robot[:, 0]
        lateral = np.abs(relative_robot[:, 1])
        relative_z = relative_robot[:, 2]

        mask = (
            (forward >= 0.0)
            & (forward <= self.config.path_length_m + self.config.safety_extension_m)
            & (lateral <= self.config.corridor_radius_m)
            & (relative_z >= self.config.min_relative_z_m)
            & (relative_z <= self.config.max_relative_z_m)
        )
        blocking_indices = np.flatnonzero(mask)
        if len(blocking_indices) < self.config.min_blocking_points:
            return None

        nearest_idx = blocking_indices[np.argmin(forward[blocking_indices])]
        return BlockedReason(
            count=len(blocking_indices),
            nearest_forward_m=float(forward[nearest_idx]),
            nearest_lateral_m=float(lateral[nearest_idx]),
            nearest_relative_z_m=float(relative_z[nearest_idx]),
        )

    @staticmethod
    def _finite_pose(pose: PoseStamped) -> bool:
        values = (
            pose.x,
            pose.y,
            pose.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        return all(math.isfinite(v) for v in values)

    def _fail(self, reason: str) -> None:
        logger.warning("FixedForwardPathPlanner did not publish path: %s", reason)
