from dimos.core.global_config import GlobalConfig
from dimos.simulation.mujoco.mujoco_process import _should_use_viewer


def test_headless_config_disables_mujoco_viewer(monkeypatch) -> None:
    monkeypatch.setenv("DISPLAY", ":0")
    assert not _should_use_viewer(GlobalConfig(viewer="none"))


def test_missing_display_disables_mujoco_viewer(monkeypatch) -> None:
    monkeypatch.delenv("DISPLAY", raising=False)
    assert not _should_use_viewer(GlobalConfig(viewer="rerun"))


def test_display_allows_mujoco_viewer(monkeypatch) -> None:
    monkeypatch.setenv("DISPLAY", ":0")
    assert _should_use_viewer(GlobalConfig(viewer="rerun"))
