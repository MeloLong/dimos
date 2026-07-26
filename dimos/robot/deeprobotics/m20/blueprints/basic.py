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

"""Basic Lynx M20 connection, TF, and Rerun blueprints."""

from typing import Any

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.protocol.pubsub.patterns import Glob
from dimos.robot.deeprobotics.m20.connection import M20Connection
from dimos.robot.deeprobotics.m20.tf import M20TF
from dimos.visualization.rerun.bridge import RerunBridgeModule
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer
from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule


def _node_edges_on_surface(msg: Any) -> Any:
    # LineSegments3D.to_rerun() defaults to z_offset=1.7 (eye-level lift), which
    # floats the planner graph ~1.7 m above the surface. Render it flat instead.
    return msg.to_rerun(z_offset=0.0)


def _raw_path_for_rerun(msg: Any) -> Any:
    """Render the unmodified A* path above the controller path."""
    if not msg.poses:
        return None
    return msg.to_rerun(color=(255, 170, 0), z_offset=0.55, radii=0.035)


def _smooth_path_for_rerun(msg: Any) -> Any:
    """Render the path sent to LocalPlanner in the existing green style."""
    if not msg.poses:
        return None
    return msg.to_rerun(color=(0, 255, 128), z_offset=0.60, radii=0.05)


def _sim_pointcloud_for_rerun(msg: Any) -> Any:
    """Reduce display-only point density without changing navigation data."""
    return msg.voxel_downsample(0.10).to_rerun(voxel_size=0.10, mode="points")


def _build_m20_rerun_blueprint(*, include_rear_camera: bool) -> Any:
    import rerun as rr
    import rerun.blueprint as rrb

    camera_views = [
        rrb.Spatial2DView(origin="world/color_image", name="M20 Front"),
    ]
    if include_rear_camera:
        camera_views.append(rrb.Spatial2DView(origin="world/color_image_rear", name="M20 Rear"))

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Vertical(*camera_views),
            rrb.Spatial3DView(
                origin="world",
                name="3D",
                contents=[
                    "+ $origin/**",
                    "- $origin/color_image",
                    "- $origin/color_image_rear",
                ],
                background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
                line_grid=rrb.LineGrid3D(
                    plane=rr.components.Plane3D.XY.with_distance(0.5),
                ),
            ),
            column_shares=[1, 2],
        ),
        rrb.TimePanel(state="hidden"),
        rrb.SelectionPanel(state="hidden"),
    )


def m20_rerun_blueprint() -> Any:
    return _build_m20_rerun_blueprint(include_rear_camera=True)


def m20_sim_rerun_blueprint() -> Any:
    return _build_m20_rerun_blueprint(include_rear_camera=False)


def build_m20_rerun(*, simulation: bool = False) -> Blueprint:
    max_hz = {
        "world/color_image": 0 if simulation else 20,
        "world/color_image_rear": 0 if simulation else 20,
        "world/slam_aligned_points": 2.0 if simulation else 10.0,
        "world/global_map": 0.2 if simulation else 1.0,
        "world/local_map": 1.0 if simulation else 2.0,
    }
    low_fps_warn = {
        "world/color_image": 7.0 if simulation else 20.0,
        "world/color_image_rear": 7.0 if simulation else 20.0,
        "world/slam_aligned_points": 1.8 if simulation else 9.8,
        "world/local_map": 0.9 if simulation else 4.5,
        "world/global_map": 0.18 if simulation else 0.8,
    }
    visual_override = {
        "world/node_edges": _node_edges_on_surface,
        "world/raw_path": _raw_path_for_rerun,
        "world/path": _smooth_path_for_rerun,
    }
    if simulation:
        visual_override.update(
            {
                "world/slam_aligned_points": _sim_pointcloud_for_rerun,
                "world/local_map": _sim_pointcloud_for_rerun,
                "world/global_map": _sim_pointcloud_for_rerun,
            }
        )

    return autoconnect(
        RerunBridgeModule.blueprint(
            blueprint=m20_sim_rerun_blueprint if simulation else m20_rerun_blueprint,
            memory_limit="2GB",
            max_hz=max_hz,
            latest_only_entities=[
                "world/slam_aligned_points",
                "world/local_map",
                "world/global_map",
                "world/global_costmap",
            ],
            use_message_timestamps=False,
            debug_stats=True,
            debug_stats_interval=5.0,
            debug_stats_entities=[
                "world/color_image",
                "world/color_image_rear",
                "world/slam_aligned_points",
                "world/global_map",
                "world/local_map",
                Glob("world/**image**"),
                Glob("world/**map**"),
                Glob("world/**point**"),
                Glob("world/**costmap**"),
            ],
            debug_low_fps_warn=low_fps_warn,
            visual_override=visual_override,
        ),
        RerunWebSocketServer.blueprint(),
        WebsocketVisModule.blueprint(),
    )


rerun = build_m20_rerun()


m20 = autoconnect(
    rerun,
    # M20TF turns the SLAM odometry into the map->base_link TF. The bridge
    # publishes it on ``slam_odom`` (not the default ``odometry``), so remap.
    M20Connection.blueprint(),
    M20TF.blueprint().remappings([(M20TF, "odometry", "slam_odom")]),
).global_config(n_workers=3)
