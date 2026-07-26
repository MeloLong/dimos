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

import numpy as np
from pydantic import ValidationError
import pytest

from dimos.core.global_config import GlobalConfig
from dimos.simulation.mujoco.depth_camera import depth_image_to_point_cloud
from dimos.simulation.mujoco.mujoco_process import (
    _hemispherical_ray_directions,
    _should_use_viewer,
)
from dimos.simulation.mujoco.sensor_config import MujocoSensorConfig
from dimos.simulation.mujoco.shared_memory import ShmReader, ShmWriter


def test_headless_config_disables_mujoco_viewer(monkeypatch) -> None:
    monkeypatch.setenv("DISPLAY", ":0")
    assert not _should_use_viewer(GlobalConfig(viewer="none"))


def test_missing_display_disables_mujoco_viewer(monkeypatch) -> None:
    monkeypatch.delenv("DISPLAY", raising=False)
    assert not _should_use_viewer(GlobalConfig(viewer="rerun"))


def test_display_allows_mujoco_viewer(monkeypatch) -> None:
    monkeypatch.setenv("DISPLAY", ":0")
    assert _should_use_viewer(GlobalConfig(viewer="rerun"))


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("width", 0),
        ("height", -1),
        ("fps", 0),
        ("pointcloud_fps", 0),
        ("pointcloud_width", 0),
        ("pointcloud_height", -1),
        ("pointcloud_min_range_m", 0),
        ("pointcloud_max_range_m", 0),
        ("pointcloud_voxel_size", 0),
    ],
)
def test_sensor_config_rejects_non_positive_values(field, value) -> None:
    with pytest.raises(ValidationError):
        MujocoSensorConfig(**{field: value})


@pytest.mark.parametrize("groups", [(), (-1,), (6,)])
def test_sensor_config_rejects_invalid_pointcloud_geom_groups(groups) -> None:
    with pytest.raises(ValidationError):
        MujocoSensorConfig(pointcloud_geom_groups=groups)


def test_sensor_config_keeps_legacy_mujoco_visible_groups_by_default() -> None:
    assert MujocoSensorConfig().pointcloud_geom_groups == (0, 1, 2)


def test_sensor_config_rejects_inverted_pointcloud_range() -> None:
    with pytest.raises(ValidationError):
        MujocoSensorConfig(pointcloud_min_range_m=5.0, pointcloud_max_range_m=5.0)


def test_airy_ray_pattern_covers_camera_forward_180_by_90_degrees() -> None:
    directions = _hemispherical_ray_directions(192, 96, 90.0)

    assert directions.shape == (192 * 96, 3)
    np.testing.assert_allclose(np.linalg.norm(directions, axis=1), 1.0, atol=1e-12)
    elevations = np.degrees(np.arcsin(directions[:, 1]))
    assert elevations.min() == pytest.approx(-45.0)
    assert elevations.max() == pytest.approx(45.0)
    assert directions[:, 0].min() < -0.999
    assert directions[:, 0].max() > 0.999
    assert directions[:, 2].min() < -0.999
    assert directions[:, 2].max() <= 1e-12


def test_pointcloud_scan_shape_is_independent_from_rgb() -> None:
    config = MujocoSensorConfig(
        width=640,
        height=360,
        pointcloud_width=192,
        pointcloud_height=96,
    )

    assert (config.width, config.height) == (640, 360)
    assert (config.pointcloud_width, config.pointcloud_height) == (192, 96)


def test_depth_point_filter_uses_configured_max_range() -> None:
    depth = np.zeros((2, 2), dtype=np.float32)
    depth[1, 1] = 4.0
    camera_pos = np.zeros(3)
    camera_mat = np.eye(3)

    legacy_points = depth_image_to_point_cloud(depth, camera_pos, camera_mat)
    extended_points = depth_image_to_point_cloud(
        depth,
        camera_pos,
        camera_mat,
        max_range_m=5.0,
    )

    assert legacy_points.shape == (0, 3)
    assert extended_points.shape == (1, 3)
    np.testing.assert_allclose(extended_points[0], [0.0, 0.0, -4.0])


def test_depth_point_filter_uses_radial_range() -> None:
    depth = np.zeros((2, 4), dtype=np.float32)
    depth[1, 0] = 4.0

    points = depth_image_to_point_cloud(
        depth,
        np.zeros(3),
        np.eye(3),
        fov_degrees=53.130102,
        max_range_m=5.0,
    )

    assert points.shape == (0, 3)


def test_video_shared_memory_uses_configured_shape() -> None:
    config = MujocoSensorConfig(width=4, height=3)
    writer = ShmWriter(config)
    reader = ShmReader(writer.shm.to_names(), config)
    pixels = np.arange(4 * 3 * 3, dtype=np.uint8).reshape(3, 4, 3)

    try:
        reader.write_video(pixels)
        frame, sequence = writer.read_video()
        assert sequence == 1
        np.testing.assert_array_equal(frame, pixels)
    finally:
        reader.cleanup()
        writer.cleanup()


def test_disabled_color_uses_minimal_video_shared_memory() -> None:
    writer = ShmWriter(MujocoSensorConfig(enable_color=False))
    try:
        assert writer.shm.video.size == 1
    finally:
        writer.cleanup()
