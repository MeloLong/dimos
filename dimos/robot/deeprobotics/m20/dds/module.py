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

"""NativeModule wrapper for the DeepRobotics M20 drdds -> LCM bridge.

Runs on the M20 itself. The robot's onboard stack publishes standard ROS 2
message types on a Fast-DDS fork ("drdds", under the ROS ``rt/`` namespace);
the C++ binary in ``cpp/`` subscribes to the topics we care about and
republishes them as the matching ``dimos_lcm`` types, which surface here as
ordinary typed ``Out`` ports. This is the onboard counterpart to
:class:`~dimos.robot.deeprobotics.m20.connection.M20Connection`, which only
reaches the high-level PatrolDevice protocol (no lidar / odometry).

Usage::

    from dimos.robot.deeprobotics.m20.dds.module import M20Dds
    from dimos.core.coordination.blueprints import autoconnect
    from dimos.core.coordination.module_coordinator import ModuleCoordinator
    from dimos.visualization.vis_module import vis_module

    ModuleCoordinator.build(autoconnect(
        M20Dds.blueprint(),
        vis_module("rerun"),
    )).loop()
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from dimos.core.native_module import LogFormat, NativeModule, NativeModuleConfig
from dimos.core.stream import Out
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.spec import perception


class M20DdsConfig(NativeModuleConfig):
    cwd: str | None = "cpp"
    executable: str = "build/m20_dds_bridge"
    # Plain CMake build (no Nix). Fetches dimos_lcm + links the drdds SDK from
    # /usr/local. Re-run is cheap once the build dir is warm.
    build_command: str | None = "cmake -B build -S . && cmake --build build -j"
    # The binary logs plain text to stderr, not structured JSON.
    log_format: LogFormat = LogFormat.TEXT

    # drdds domain + optional NIC (name or IP). Empty NIC = SDK default.
    domain: int = 0
    network: str | None = None

    # drdds source topics (ROS names; the bridge prepends the `rt/` DDS prefix).
    # Defaults match the M20's onboard topic map (see `drlist`).
    lidar_topic: str = "/LIDAR/POINTS"
    imu_topic: str = "/IMU"
    odom_topic: str = "/ODOM"

    # Optional frame_id override; empty preserves each sample's own header frame.
    frame_id: str | None = None


class M20Dds(NativeModule, perception.Lidar, perception.IMU, perception.Odometry):
    """Bridges the M20's onboard drdds bus onto the dimos LCM bus."""

    config: M20DdsConfig

    lidar: Out[PointCloud2]
    imu: Out[Imu]
    odometry: Out[Odometry]


# Verify protocol port compliance (mypy will flag missing ports).
if TYPE_CHECKING:
    M20Dds()
