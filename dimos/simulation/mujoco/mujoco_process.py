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

import base64
from contextlib import nullcontext
import json
import os
import pickle
import signal
import sys
import time
from typing import Any

import mujoco
from mujoco import viewer
import numpy as np
from numpy.typing import NDArray

from dimos.core.global_config import GlobalConfig
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.simulation.mujoco.depth_camera import depth_image_to_point_cloud
from dimos.simulation.mujoco.model import load_model, load_scene_xml
from dimos.simulation.mujoco.person_on_track import PersonPositionController
from dimos.simulation.mujoco.sensor_config import MujocoSensorConfig
from dimos.simulation.mujoco.shared_memory import ShmReader
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def _should_use_viewer(config: GlobalConfig) -> bool:
    return config.viewer != "none" and bool(os.environ.get("DISPLAY"))


class MockController:
    """Controller that reads commands from shared memory."""

    def __init__(self, shm_interface: ShmReader) -> None:
        self.shm = shm_interface
        self._command = np.zeros(3, dtype=np.float32)

    def get_command(self) -> NDArray[Any]:
        """Get the current movement command."""
        cmd_data = self.shm.read_command()
        if cmd_data is not None:
            linear, angular = cmd_data
            # MuJoCo expects [forward, lateral, rotational]
            self._command[0] = linear[0]  # forward/backward
            self._command[1] = linear[1]  # left/right
            self._command[2] = angular[2]  # rotation
        result: NDArray[Any] = self._command.copy()
        return result

    def stop(self) -> None:
        """Stop method to satisfy InputController protocol."""
        pass


def _camera_id(model: mujoco.MjModel, camera_name: str) -> int:
    camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
    if camera_id < 0:
        raise ValueError(f"MuJoCo camera {camera_name!r} does not exist in the model")
    return camera_id


def _run_simulation(
    config: GlobalConfig,
    shm: ShmReader,
    sensor_config: MujocoSensorConfig | None = None,
) -> None:
    sensor_config = sensor_config or MujocoSensorConfig()
    robot_name = config.robot_model or "unitree_go1"
    if robot_name == "unitree_go2":
        robot_name = "unitree_go1"

    controller = MockController(shm)
    model, data = load_model(
        controller,
        robot=robot_name,
        scene_xml=load_scene_xml(config),
        person_collision_enabled=config.mujoco_person_collision_enabled,
    )

    if model is None or data is None:
        raise ValueError("Failed to load MuJoCo model: model or data is None")

    match robot_name:
        case "unitree_go1":
            z = 0.3
        case "unitree_g1":
            z = 0.8
        case _:
            z = 0

    start_pos = config.mujoco_start_pos_float

    data.qpos[0:3] = [start_pos[0], start_pos[1], z]

    mujoco.mj_forward(model, data)

    person_position_controller = PersonPositionController(model)

    viewer_context = (
        viewer.launch_passive(model, data, show_left_ui=False, show_right_ui=False)
        if _should_use_viewer(config)
        else nullcontext(None)
    )

    with viewer_context as m_viewer:
        rgb_renderer = None
        rgb_camera_id = None
        if sensor_config.enable_color:
            rgb_camera_id = _camera_id(model, sensor_config.color_camera_name)
            rgb_renderer = mujoco.Renderer(
                model,
                height=sensor_config.height,
                width=sensor_config.width,
            )

        pointcloud_renderers: list[tuple[mujoco.Renderer, int]] = []
        if sensor_config.enable_pointcloud:
            for camera_name in sensor_config.pointcloud_camera_names:
                renderer = mujoco.Renderer(
                    model,
                    height=sensor_config.height,
                    width=sensor_config.width,
                )
                renderer.enable_depth_rendering()
                pointcloud_renderers.append((renderer, _camera_id(model, camera_name)))

            import open3d as o3d  # type: ignore[import-untyped]

        color_scene_option = mujoco.MjvOption()
        pointcloud_scene_option = mujoco.MjvOption()
        pointcloud_scene_option.geomgroup[:] = 0
        pointcloud_scene_option.geomgroup[list(sensor_config.pointcloud_geom_groups)] = 1

        # Timing control
        last_video_time = float("-inf")
        last_pointcloud_time = float("-inf")
        video_interval = 1.0 / sensor_config.fps
        pointcloud_interval = 1.0 / sensor_config.pointcloud_fps
        simulation_interval = model.opt.timestep * config.mujoco_steps_per_frame

        if m_viewer is not None:
            m_viewer.cam.lookat = config.mujoco_camera_position_float[0:3]
            m_viewer.cam.distance = config.mujoco_camera_position_float[3]
            m_viewer.cam.azimuth = config.mujoco_camera_position_float[4]
            m_viewer.cam.elevation = config.mujoco_camera_position_float[5]

        shm.signal_ready()

        try:
            while (m_viewer is None or m_viewer.is_running()) and not shm.should_stop():
                step_start = time.monotonic()

                for _ in range(config.mujoco_steps_per_frame):
                    mujoco.mj_step(model, data)

                person_position_controller.tick(data)

                if m_viewer is not None:
                    m_viewer.sync()

                pos = data.qpos[0:3].copy()
                quat = data.qpos[3:7].copy()  # (w, x, y, z)
                shm.write_odom(pos, quat, time.time())

                current_time = time.monotonic()

                if (
                    rgb_renderer is not None
                    and rgb_camera_id is not None
                    and current_time - last_video_time >= video_interval
                ):
                    rgb_renderer.update_scene(
                        data,
                        camera=rgb_camera_id,
                        scene_option=color_scene_option,
                    )
                    shm.write_video(rgb_renderer.render())
                    last_video_time = current_time

                if (
                    pointcloud_renderers
                    and current_time - last_pointcloud_time >= pointcloud_interval
                ):
                    all_points = []
                    for renderer, camera_id in pointcloud_renderers:
                        renderer.update_scene(
                            data,
                            camera=camera_id,
                            scene_option=pointcloud_scene_option,
                        )
                        points = depth_image_to_point_cloud(
                            renderer.render(),
                            data.cam_xpos[camera_id],
                            data.cam_xmat[camera_id].reshape(3, 3),
                            fov_degrees=sensor_config.pointcloud_fov_deg,
                        )
                        if points.size > 0:
                            all_points.append(points)

                    if all_points:
                        pcd = o3d.geometry.PointCloud()
                        pcd.points = o3d.utility.Vector3dVector(np.vstack(all_points))
                        pcd = pcd.voxel_down_sample(voxel_size=sensor_config.pointcloud_voxel_size)
                        shm.write_lidar(
                            PointCloud2(pointcloud=pcd, ts=time.time(), frame_id="world")
                        )

                    last_pointcloud_time = current_time

                sleep_time = simulation_interval - (time.monotonic() - step_start)
                if sleep_time > 0:
                    time.sleep(sleep_time)
        finally:
            person_position_controller.stop()
            if rgb_renderer is not None:
                rgb_renderer.close()
            for renderer, _ in pointcloud_renderers:
                renderer.close()


if __name__ == "__main__":
    global_config = pickle.loads(base64.b64decode(sys.argv[1]))
    shm_names = json.loads(sys.argv[2])
    sensor_config = (
        pickle.loads(base64.b64decode(sys.argv[3])) if len(sys.argv) > 3 else MujocoSensorConfig()
    )

    shm = ShmReader(shm_names, sensor_config)

    def signal_handler(_signum: int, _frame: Any) -> None:
        # Signal the main loop to exit gracefully so the viewer context
        # manager can close the window and clean up resources.
        shm.signal_stop()

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        _run_simulation(global_config, shm, sensor_config)
    finally:
        shm.cleanup()
