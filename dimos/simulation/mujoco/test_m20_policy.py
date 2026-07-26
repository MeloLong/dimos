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

from pathlib import Path

import mujoco
import numpy as np

from dimos.simulation.mujoco.mujoco_process import _hemispherical_ray_directions
from dimos.simulation.mujoco.policy import M20OnnxController


class _ZeroCommand:
    def get_command(self) -> np.ndarray:
        return np.zeros(3, dtype=np.float32)

    def stop(self) -> None:
        pass


def _asset_dir() -> Path:
    return Path(__file__).resolve().parents[2] / "robot/deeprobotics/m20/assets"


def _load_model() -> tuple[mujoco.MjModel, mujoco.MjData]:
    model = mujoco.MjModel.from_xml_path(str(_asset_dir() / "deeprobotics_m20.xml"))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    mujoco.mj_forward(model, data)
    return model, data


def test_official_m20_model_contract() -> None:
    model, data = _load_model()

    assert model.nq == 23
    assert model.nv == 22
    assert model.nu == 16
    assert model.key("home").qpos.shape == (23,)
    assert data.qpos[2] == 0.58
    assert [model.actuator(i).name for i in range(model.nu)] == [
        "fl_hipx_joint",
        "fl_hipy_joint",
        "fl_knee_joint",
        "fl_wheel_joint",
        "fr_hipx_joint",
        "fr_hipy_joint",
        "fr_knee_joint",
        "fr_wheel_joint",
        "hl_hipx_joint",
        "hl_hipy_joint",
        "hl_knee_joint",
        "hl_wheel_joint",
        "hr_hipx_joint",
        "hr_hipy_joint",
        "hr_knee_joint",
        "hr_wheel_joint",
    ]
    for name in (
        "head_camera",
        "lidar_front_camera",
        "lidar_rear_camera",
        "lidar_left_camera",
        "lidar_right_camera",
    ):
        assert model.camera(name).id >= 0


def test_m20_front_and_rear_depth_cameras_face_opposite_directions() -> None:
    model, data = _load_model()
    front_id = model.camera("lidar_front_camera").id
    rear_id = model.camera("lidar_rear_camera").id

    front_view = -data.cam_xmat[front_id].reshape(3, 3)[:, 2]
    rear_view = -data.cam_xmat[rear_id].reshape(3, 3)[:, 2]

    np.testing.assert_allclose(front_view, [1.0, 0.0, 0.0], atol=1e-7)
    np.testing.assert_allclose(rear_view, [-1.0, 0.0, 0.0], atol=1e-7)


def test_m20_front_and_rear_airy_sectors_point_outward() -> None:
    model, data = _load_model()
    directions = _hemispherical_ray_directions(128, 96, 90.0)
    front_id = model.camera("lidar_front_camera").id
    rear_id = model.camera("lidar_rear_camera").id

    front_directions = directions @ data.cam_xmat[front_id].reshape(3, 3).T
    rear_directions = directions @ data.cam_xmat[rear_id].reshape(3, 3).T

    assert front_directions[:, 0].min() >= -1e-12
    assert rear_directions[:, 0].max() <= 1e-12


def test_m20_policy_observation_and_output_contract() -> None:
    model, data = _load_model()
    controller = M20OnnxController(
        policy_path=str(_asset_dir() / "deeprobotics_m20_policy.onnx"),
        default_angles=data.qpos[7:].copy(),
        n_substeps=20,
        action_scale=1.0,
        input_controller=_ZeroCommand(),
        ctrl_dt=0.02,
    )

    assert controller.get_obs(model, data).shape == (57,)
    for _ in range(20):
        controller.get_control(model, data)
        mujoco.mj_step(model, data)

    assert data.ctrl.shape == (16,)
    assert np.isfinite(data.ctrl).all()
