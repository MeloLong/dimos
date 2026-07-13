import numpy as np
from pydantic import ValidationError
import pytest

from dimos.core.global_config import GlobalConfig
from dimos.simulation.mujoco.mujoco_process import _should_use_viewer
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
        ("pointcloud_voxel_size", 0),
    ],
)
def test_sensor_config_rejects_non_positive_values(field, value) -> None:
    with pytest.raises(ValidationError):
        MujocoSensorConfig(**{field: value})


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
