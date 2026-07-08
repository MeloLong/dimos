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

"""Hosted teleop blueprints (WebRTC transport)."""

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.core.transport import (
    CloudflareTransport,
    CloudflareVideoTransport,
    LCMTransport,
    LiveKitTransport,
    LiveKitVideoTransport,
)
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.voxels import VoxelGridMapper
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos.robot.manipulators.xarm.blueprints.teleop import (
    coordinator_teleop_xarm6,
    coordinator_teleop_xarm7,
)
from dimos.robot.manipulators.xarm.config import XARM7_SIM_PATH
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import unitree_go2_basic
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.teleop.quest.quest_types import Buttons
from dimos.teleop.quest_hosted.arm_hosted_connection import ArmHostedConnection
from dimos.teleop.quest_hosted.go2_hosted_connection import Go2HostedConnection
from dimos.teleop.quest_hosted.hosted_extensions import HostedTwistTeleopModule

# Hosted teleop via the legacy module wrapper (the transport swap is preferred).
teleop_hosted_go2 = autoconnect(
    HostedTwistTeleopModule.blueprint(),
    unitree_go2_basic,
).global_config(n_workers=8, viewer="none")

# Hosted teleop over CF Realtime. Run with -o transports.broker.api_key=dtk_live_...
# VoxelGridMapper → CostMapper build the OccupancyGrid; Go2HostedConnection
# encodes it (+ odom) onto state_reliable_back for the operator minimap.
teleop_hosted_go2_transport = (
    autoconnect(
        unitree_go2_basic.disabled_modules(GO2Connection),
        Go2HostedConnection.blueprint(),
        VoxelGridMapper.blueprint(emit_every=5),
        CostMapper.blueprint(),
        # Click-to-navigate: goal_request (operator map click) → planned path →
        # nav_cmd_vel back into the hosted connection (yields to live WASD).
        ReplanningAStarPlanner.blueprint(),
    )
    .transports(
        {
            ("cmd_vel", Twist): CloudflareTransport.spec("cmd_unreliable", TwistStamped),
            # mux_image (not color_image): the muxed out-frame carries the
            # latency stamp and works for 1 cam (mux returns cam1 alone) or more.
            ("mux_image", Image): CloudflareVideoTransport.spec(),
            ("state_json", bytes): CloudflareTransport.spec("state_reliable"),
            ("telemetry_out", bytes): CloudflareTransport.spec("state_reliable_back"),
            ("cmd_raw", bytes): CloudflareTransport.spec("cmd_unreliable"),  # stats tap
            ("cmd_vel_stamped", TwistStamped): LCMTransport.spec("cmd_vel_stamped", TwistStamped),
            # Map chain over LCM (in-process); the compressed map + odom go to the
            # operator on their own unreliable channel (room to grow to pointclouds).
            ("lidar", PointCloud2): LCMTransport.spec("lidar", PointCloud2),
            ("global_map", PointCloud2): LCMTransport.spec("global_map", PointCloud2),
            ("global_costmap", OccupancyGrid): LCMTransport.spec("global_costmap", OccupancyGrid),
            ("map_out", bytes): CloudflareTransport.spec("map_unreliable"),
        }
    )
    .global_config(viewer="none")
)


# LiveKit twin of teleop_hosted_go2_transport — same channels, LiveKit SFU.
# Run with -o transports.broker.api_key=dtk_live_...
teleop_hosted_go2_livekit = (
    autoconnect(
        unitree_go2_basic.disabled_modules(GO2Connection),
        Go2HostedConnection.blueprint(),
    )
    .transports(
        {
            ("cmd_vel", Twist): LiveKitTransport.spec("cmd_unreliable", TwistStamped),
            ("color_image", Image): LiveKitVideoTransport.spec(),
            ("state_json", bytes): LiveKitTransport.spec("state_reliable"),
            ("telemetry_out", bytes): LiveKitTransport.spec("state_reliable_back"),
            ("cmd_raw", bytes): LiveKitTransport.spec("cmd_unreliable"),  # stats tap
            ("cmd_vel_stamped", TwistStamped): LiveKitTransport.spec(
                "cmd_unreliable", TwistStamped
            ),
        }
    )
    .global_config(viewer="none")
)


# Adds a RealSense as cam2 (mux'd into the video track by Go2HostedConnection).
# Needs the RealSense wired in; use teleop-hosted-go2-transport otherwise.
# Same map overlay as teleop_hosted_go2_transport.
teleop_hosted_go2_multicam = (
    autoconnect(
        unitree_go2_basic.disabled_modules(GO2Connection),
        Go2HostedConnection.blueprint(),
        RealSenseCamera.blueprint(enable_depth=False, enable_pointcloud=False),
        VoxelGridMapper.blueprint(emit_every=5),
        CostMapper.blueprint(),
        ReplanningAStarPlanner.blueprint(),
    )
    .remappings([(RealSenseCamera, "color_image", "cam2_in")])
    .transports(
        {
            ("cmd_vel", Twist): CloudflareTransport.spec("cmd_unreliable", TwistStamped),
            ("mux_image", Image): CloudflareVideoTransport.spec(),
            ("cam2_in", Image): LCMTransport.spec("cam2_in", Image),
            ("state_json", bytes): CloudflareTransport.spec("state_reliable"),
            ("telemetry_out", bytes): CloudflareTransport.spec("state_reliable_back"),
            ("cmd_raw", bytes): CloudflareTransport.spec("cmd_unreliable"),
            # LCM, not CF: cmd_vel_stamped is the robot re-publishing the decoded
            # operator cmd for the local recorder. cmd_unreliable is an
            # operator→robot channel; publishing there raises in the broker.
            ("cmd_vel_stamped", TwistStamped): LCMTransport.spec("cmd_vel_stamped", TwistStamped),
            # Map chain over LCM (same as the transport blueprint).
            ("lidar", PointCloud2): LCMTransport.spec("lidar", PointCloud2),
            ("global_map", PointCloud2): LCMTransport.spec("global_map", PointCloud2),
            ("global_costmap", OccupancyGrid): LCMTransport.spec("global_costmap", OccupancyGrid),
            ("map_out", bytes): CloudflareTransport.spec("map_unreliable"),
        }
    )
    .global_config(viewer="none")
)


# ─── XArm hosted manipulation (coordinator-driven, WebXR operator) ──────────
#
# ArmHostedConnection bridges operator VR poses/buttons (over cmd_unreliable) to
# the ControlCoordinator's TeleopIKTask (over LCM), and mux'es the arm cameras
# into one CF video track. All broker-bound streams live on ArmHostedConnection
# so they share the one BrokerProvider session. Run any of these with
# `-o transports.broker.api_key=dtk_live_...`; pass `--simulation` for MuJoCo.


# Distinct classes so two RealSense units coexist in one blueprint: module
# identity is the class throughout the stack (blueprint dedup, coordinator
# registries, remap keys, RPC topics, config namespace). Configure serials with
# -o frontcamera.serial_number=... -o wristcamera.serial_number=...
class FrontCamera(RealSenseCamera):
    pass


class WristCamera(RealSenseCamera):
    pass


# Broker-bound streams for ArmHostedConnection — all live on the one module so
# they share the single BrokerProvider session.
_ARM_BROKER_TRANSPORTS = {
    ("cmd_raw", bytes): CloudflareTransport.spec("cmd_unreliable"),
    ("state_json", bytes): CloudflareTransport.spec("state_reliable"),
    ("telemetry_out", bytes): CloudflareTransport.spec("state_reliable_back"),
    ("mux_image", Image): CloudflareVideoTransport.spec(),
    # Station → coordinator (LCM, cross-worker). right_controller_output carries
    # the delta pose stamped with the task name (routes to the TeleopIKTask);
    # teleop_buttons carries the engage / analog-trigger state.
    ("right_controller_output", PoseStamped): LCMTransport.spec(
        "coordinator_cartesian_command", PoseStamped
    ),
    ("teleop_buttons", Buttons): LCMTransport.spec("teleop_buttons", Buttons),
}


# Single-camera XArm hosted teleop: one front RealSense mux'd to the video track.
def _teleop_hosted_xarm(coordinator):
    return (
        autoconnect(
            ArmHostedConnection.blueprint(task_names={"right": "teleop_xarm"}),
            coordinator,
            FrontCamera.blueprint(camera_name="front", enable_depth=False, enable_pointcloud=False),
        )
        .remappings([(FrontCamera, "color_image", "cam1_in")])
        .transports(
            {
                **_ARM_BROKER_TRANSPORTS,
                ("cam1_in", Image): LCMTransport.spec("cam1_in", Image),
            }
        )
        .global_config(viewer="none")
    )


teleop_hosted_xarm6 = _teleop_hosted_xarm(coordinator_teleop_xarm6)
teleop_hosted_xarm7 = _teleop_hosted_xarm(coordinator_teleop_xarm7)


# Two-camera XArm hosted teleop: front/overview = cam1, wrist = cam2,
# operator-selectable via camera_select, mux'd into one video track.
#
# Real: two RealSense units (-o frontcamera.serial_number=... etc.).
# --simulation (sim_path with named cameras only): MuJoCo renders both — the
# coordinator already brings a bare MujocoSimModule; we override it (same class
# = same module) with one that also renders the overview cam. env_camera →
# cam1, wrist_camera → cam2. 848×480 matches RealSense and stays wide enough
# for the latency stamp (needs ≥768px).
def _teleop_hosted_xarm_multicam(coordinator, *, sim_path=None, dof=7):
    if global_config.simulation and sim_path is not None:
        cameras = (
            MujocoSimModule.blueprint(
                address=str(sim_path),
                headless=False,
                dof=dof,
                camera_name="wrist_camera",
                camera2_name="env_camera",
                width=848,
                height=480,
            ),
        )
        remaps = [
            (MujocoSimModule, "color_image2", "cam1_in"),  # env overview → cam1
            (MujocoSimModule, "color_image", "cam2_in"),  # wrist → cam2
        ]
    else:
        cameras = (
            FrontCamera.blueprint(camera_name="front", enable_depth=False, enable_pointcloud=False),
            WristCamera.blueprint(camera_name="wrist", enable_depth=False, enable_pointcloud=False),
        )
        remaps = [
            (FrontCamera, "color_image", "cam1_in"),
            (WristCamera, "color_image", "cam2_in"),
        ]

    return (
        autoconnect(
            ArmHostedConnection.blueprint(task_names={"right": "teleop_xarm"}),
            coordinator,
            *cameras,
        )
        .remappings(remaps)
        .transports(
            {
                **_ARM_BROKER_TRANSPORTS,
                # Cameras → station over LCM (cameras run in other workers).
                ("cam1_in", Image): LCMTransport.spec("cam1_in", Image),
                ("cam2_in", Image): LCMTransport.spec("cam2_in", Image),
            }
        )
        .global_config(viewer="none")
    )


# xArm6 sim has no cameras in its MJCF (data/xarm6/scene.xml) — real-only until
# env_camera/wrist_camera are added there. xArm7 sim renders both.
teleop_hosted_xarm6_multicam = _teleop_hosted_xarm_multicam(coordinator_teleop_xarm6, dof=6)
teleop_hosted_xarm7_multicam = _teleop_hosted_xarm_multicam(
    coordinator_teleop_xarm7, sim_path=XARM7_SIM_PATH, dof=7
)


__all__ = [
    "teleop_hosted_go2",
    "teleop_hosted_go2_livekit",
    "teleop_hosted_go2_multicam",
    "teleop_hosted_go2_transport",
    "teleop_hosted_xarm6",
    "teleop_hosted_xarm6_multicam",
    "teleop_hosted_xarm7",
    "teleop_hosted_xarm7_multicam",
]
