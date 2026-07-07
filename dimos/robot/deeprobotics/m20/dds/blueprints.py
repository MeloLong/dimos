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

"""Blueprints for the M20 onboard drdds bridge."""

from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.deeprobotics.m20.dds.module import M20Dds
from dimos.visualization.vis_module import vis_module

# Bridge the onboard lidar / imu / odometry onto LCM and view it in rerun.
m20_dds_rerun = autoconnect(
    M20Dds.blueprint(),
    vis_module("rerun"),
).global_config(n_workers=2, robot_model="m20_dds")
