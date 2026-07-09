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

"""Fixed pattern path planner for M20 bringup/debugging.

This module is intentionally much simpler than MLSPlannerNative: when a goal is
received, it ignores the goal position and cycles through test paths in the
robot frame: forward, backward, left, right, and a circle. The path is only
published when the latest local map has no occupied voxels inside the configured
corridor swept by that path.
"""

from __future__ import annotations

from dataclasses import dataclass
from itertools import pairwise
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
    circle_radius_m: float = 1
    circle_end_gap_m: float = 0.3
    safety_extension_m: float = 0.3
    sample_spacing_m: float = 0.2
    corridor_radius_m: float = 0.6
    circle_corridor_radius_m: float = 0.25
    min_relative_z_m: float = -0.2
    max_relative_z_m: float = 1.2
    min_blocking_points: int = 3


@dataclass(frozen=True)
class BlockedReason:
    direction: str
    count: int
    nearest_along_m: float
    nearest_lateral_m: float
    nearest_relative_z_m: float
    nearest_angle_deg: float | None = None

    def format(self) -> str:
        location = f"along={self.nearest_along_m:.2f}m"
        if self.nearest_angle_deg is not None:
            location += f" ({self.nearest_angle_deg:.1f}deg)"
        return (
            f"{self.count} occupied point(s) in {self.direction} corridor; "
            f"nearest at {location}, "
            f"lateral={self.nearest_lateral_m:.2f}m, "
            f"relative_z={self.nearest_relative_z_m:.2f}m"
        )


@dataclass(frozen=True)
class PathDirection:
    name: str
    x: float
    y: float
    circle: bool = False


class FixedForwardPathPlanner(Module):
    """Publish a fixed test path in a cycling robot-frame pattern on goal.

    The ports mirror MLSPlannerNative's public contract so this module can be
    swapped into M20 navigation blueprints with minimal remapping changes.
    """

    _directions = (
        PathDirection("forward", 1.0, 0.0),
        PathDirection("backward", -1.0, 0.0),
        PathDirection("left", 0.0, 1.0),
        PathDirection("right", 0.0, -1.0),
        PathDirection("circle", 0.0, 0.0, circle=True),
    )

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
        self._goal_count = 0

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

        direction = self._next_direction()
        path = self._make_path(start, direction)
        blocked = self._blocked_reason(start, local_map, path, direction)
        if blocked is not None:
            self._fail(f"blocked: {blocked.format()}")
            return

        self.path.publish(path)
        end = path.poses[-1]
        path_length = self._path_length(path)
        logger.info(
            "FixedForwardPathPlanner published %s path #%d: %.2fm, %d poses, "
            "start=(%.2f, %.2f, %.2f), end=(%.2f, %.2f, %.2f), frame=%s",
            direction.name,
            self._goal_count,
            path_length,
            len(path.poses),
            start.x,
            start.y,
            start.z,
            end.x,
            end.y,
            end.z,
            path.frame_id,
        )

    def _next_direction(self) -> PathDirection:
        direction = self._directions[self._goal_count % len(self._directions)]
        self._goal_count += 1
        return direction

    def _make_path(self, start: PoseStamped, direction: PathDirection) -> Path:
        if direction.circle:
            return self._make_circle_path(start)
        return self._make_straight_path(start, direction)

    def _make_straight_path(self, start: PoseStamped, direction: PathDirection) -> Path:
        spacing = max(self.config.sample_spacing_m, 1e-3)
        steps = max(1, math.ceil(self.config.path_length_m / spacing))
        poses: list[PoseStamped] = []
        for i in range(steps + 1):
            distance = min(i * spacing, self.config.path_length_m)
            offset = start.orientation.rotate_vector(
                Vector3(direction.x * distance, direction.y * distance, 0.0)
            )
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

    def _make_circle_path(self, start: PoseStamped) -> Path:
        radius = max(self.config.circle_radius_m, 1e-3)
        spacing = max(self.config.sample_spacing_m, 1e-3)
        gap = min(max(self.config.circle_end_gap_m, 0.0), 2.0 * radius)
        angle_gap = 2.0 * math.asin(gap / (2.0 * radius)) if gap > 0.0 else 0.0
        sweep_angle = max(0.0, 2.0 * math.pi - angle_gap)
        steps = max(8, math.ceil((sweep_angle * radius) / spacing))
        poses: list[PoseStamped] = []
        for i in range(steps + 1):
            theta = -math.pi / 2.0 + (sweep_angle * i / steps)
            # Center is robot-left of the start, so the robot starts on the
            # circle and the initial tangent points along robot +X.
            robot_x = radius * math.cos(theta)
            robot_y = radius + radius * math.sin(theta)
            offset = start.orientation.rotate_vector(Vector3(robot_x, robot_y, 0.0))
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

    def _blocked_reason(
        self, start: PoseStamped, local_map: PointCloud2, path: Path, direction: PathDirection
    ) -> BlockedReason | None:
        points = local_map.points_f32()
        if points.size == 0 or len(path.poses) < 2:
            return None

        origin = np.array([start.x, start.y, start.z], dtype=np.float32)
        relative_world = points[:, :3] - origin
        rotation_inv = start.orientation.inverse().to_rotation_matrix().astype(np.float32)
        relative_robot = relative_world @ rotation_inv.T

        path_xy = self._path_xy_in_robot_frame(start, path)
        if not direction.circle:
            extension = np.array(
                [
                    [
                        direction.x * self.config.safety_extension_m,
                        direction.y * self.config.safety_extension_m,
                    ]
                ],
                dtype=np.float32,
            )
            path_xy = np.vstack([path_xy, path_xy[-1:] + extension])

        along, lateral = self._nearest_path_progress_and_distance(relative_robot[:, :2], path_xy)
        relative_z = relative_robot[:, 2]

        corridor_radius = (
            self.config.circle_corridor_radius_m
            if direction.circle
            else self.config.corridor_radius_m
        )
        mask = (
            (along >= 0.0)
            & (lateral <= corridor_radius)
            & (relative_z >= self.config.min_relative_z_m)
            & (relative_z <= self.config.max_relative_z_m)
        )
        blocking_indices = np.flatnonzero(mask)
        if len(blocking_indices) < self.config.min_blocking_points:
            return None

        nearest_idx = blocking_indices[np.argmin(along[blocking_indices])]
        return BlockedReason(
            direction=direction.name,
            count=len(blocking_indices),
            nearest_along_m=float(along[nearest_idx]),
            nearest_lateral_m=float(lateral[nearest_idx]),
            nearest_relative_z_m=float(relative_z[nearest_idx]),
            nearest_angle_deg=self._circle_angle_deg(float(along[nearest_idx]))
            if direction.circle
            else None,
        )

    def _path_xy_in_robot_frame(self, start: PoseStamped, path: Path) -> np.ndarray:
        origin = np.array([start.x, start.y, start.z], dtype=np.float32)
        path_world = np.array([[pose.x, pose.y, pose.z] for pose in path.poses], dtype=np.float32)
        rotation_inv = start.orientation.inverse().to_rotation_matrix().astype(np.float32)
        path_robot = (path_world - origin) @ rotation_inv.T
        return path_robot[:, :2]

    @staticmethod
    def _nearest_path_progress_and_distance(
        points_xy: np.ndarray, path_xy: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        best_dist2 = np.full(points_xy.shape[0], np.inf, dtype=np.float32)
        best_progress = np.zeros(points_xy.shape[0], dtype=np.float32)
        progress_at_segment = 0.0

        for start, end in pairwise(path_xy):
            segment = end - start
            segment_len = float(np.linalg.norm(segment))
            if segment_len <= 1e-6:
                continue

            relative = points_xy - start
            t = np.clip((relative @ segment) / (segment_len * segment_len), 0.0, 1.0)
            closest = start + t[:, None] * segment
            dist2 = np.sum((points_xy - closest) ** 2, axis=1)
            update = dist2 < best_dist2
            best_dist2[update] = dist2[update]
            best_progress[update] = progress_at_segment + t[update] * segment_len
            progress_at_segment += segment_len

        return best_progress, np.sqrt(best_dist2)

    @staticmethod
    def _path_length(path: Path) -> float:
        if len(path.poses) < 2:
            return 0.0
        points = np.array([[pose.x, pose.y] for pose in path.poses], dtype=np.float32)
        segments = points[1:] - points[:-1]
        return float(np.linalg.norm(segments, axis=1).sum())

    def _circle_angle_deg(self, along_m: float) -> float:
        radius = max(self.config.circle_radius_m, 1e-3)
        return math.degrees(along_m / radius) % 360.0

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
