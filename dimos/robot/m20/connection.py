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

from __future__ import annotations

from dataclasses import dataclass
import threading
import time
from typing import Any, Protocol

import numpy as np
from pydantic import Field

from dimos.agents.annotation import skill
from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


@dataclass(frozen=True)
class M20LidarFrame:
    """Raw lidar frame returned by the M20 SDK adapter."""

    points: np.ndarray
    intensities: np.ndarray | None = None
    timestamp: float | None = None
    frame_id: str = "lidar"


@dataclass(frozen=True)
class M20CameraFrame:
    """Raw camera frame returned by the M20 SDK adapter."""

    image: np.ndarray
    timestamp: float | None = None
    frame_id: str = "camera_optical"
    format: ImageFormat = ImageFormat.BGR


class M20SensorClient(Protocol):
    """Small adapter boundary around the real M20 sensor SDK.

    Keep vendor-specific SDK objects behind this interface. The DimOS module
    only needs blocking reads for the latest lidar and camera frames.
    """

    def connect(self) -> None: ...
    def close(self) -> None: ...
    def read_lidar_frame(self) -> M20LidarFrame: ...
    def read_camera_frame(self) -> M20CameraFrame: ...


class M20MockSensorClient:
    """Local fake client used until the real M20 SDK adapter is wired in."""

    def connect(self) -> None:
        #todo: Replace this mock client with a real M20 SDK adapter.
        logger.warning("Using M20MockSensorClient; replace _create_client() with real SDK setup")

    def close(self) -> None:
        pass

    def read_lidar_frame(self) -> M20LidarFrame:
        #todo: Return real SDK lidar points as an Nx3 float array, in meters.
        return M20LidarFrame(
            points=np.zeros((0, 3), dtype=np.float32),
            timestamp=time.time(),
        )

    def read_camera_frame(self) -> M20CameraFrame:
        #todo: Return real SDK camera frames and set the correct RGB/BGR format.
        return M20CameraFrame(
            image=np.zeros((480, 640, 3), dtype=np.uint8),
            timestamp=time.time(),
        )


class M20SensorConfig(ModuleConfig):
    ip: str = Field(default_factory=lambda m: m["g"].robot_ip or "192.168.1.20")
    #todo: Update these frame IDs after the M20 sensor extrinsics are finalized.
    lidar_frame_id: str = "lidar"
    camera_frame_id: str = "camera_optical"
    lidar_hz: float = 20.0
    camera_hz: float = 30.0
    #todo: Confirm the M20 camera SDK output format before using real images.
    image_format: ImageFormat = ImageFormat.BGR


class M20Sensor(Module):
    """Publish M20 sensor SDK data as DimOS lidar and camera streams."""

    dedicated_worker = True

    config: M20SensorConfig

    lidar: Out[PointCloud2]
    color_image: Out[Image]
    #todo: Add camera_info: Out[CameraInfo] once M20 intrinsics are available.

    client: M20SensorClient
    _running: bool
    _lidar_thread: threading.Thread | None
    _camera_thread: threading.Thread | None
    _latest_image: Image | None

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.client = self._create_client()
        self._running = False
        self._lidar_thread = None
        self._camera_thread = None
        self._latest_image = None

    def _create_client(self) -> M20SensorClient:
        """Create the SDK adapter.

        Replace this method with real M20 SDK initialization, or return a small
        adapter class that wraps the SDK's connect/read/close calls.
        """
        #todo: Instantiate the real M20 SDK client here, using self.config.ip.
        return M20MockSensorClient()

    @rpc
    def start(self) -> None:
        super().start()
        self.client.connect()
        self._running = True

        self._lidar_thread = threading.Thread(
            target=self._lidar_loop,
            name="m20-lidar",
            daemon=True,
        )
        self._camera_thread = threading.Thread(
            target=self._camera_loop,
            name="m20-camera",
            daemon=True,
        )

        self._lidar_thread.start()
        self._camera_thread.start()

    @rpc
    def stop(self) -> None:
        self._running = False

        if self._lidar_thread and self._lidar_thread.is_alive():
            self._lidar_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)

        if self._camera_thread and self._camera_thread.is_alive():
            self._camera_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)

        self.client.close()
        super().stop()

    def _lidar_loop(self) -> None:
        period = 1.0 / self.config.lidar_hz if self.config.lidar_hz > 0 else 0.0
        while self._running:
            started_at = time.monotonic()
            try:
                raw = self.client.read_lidar_frame()
                self.lidar.publish(self._to_pointcloud2(raw))
            except Exception:
                logger.exception("Failed to read or publish M20 lidar frame")

            self._sleep_remaining(period, started_at)

    def _camera_loop(self) -> None:
        period = 1.0 / self.config.camera_hz if self.config.camera_hz > 0 else 0.0
        while self._running:
            started_at = time.monotonic()
            try:
                raw = self.client.read_camera_frame()
                image = self._to_image(raw)
                self._latest_image = image
                self.color_image.publish(image)
            except Exception:
                logger.exception("Failed to read or publish M20 camera frame")

            self._sleep_remaining(period, started_at)

    def _to_pointcloud2(self, raw: M20LidarFrame) -> PointCloud2:
        """Convert SDK lidar output to DimOS PointCloud2.

        Add lidar post-processing here: unit conversion, coordinate transform,
        crop/filter, deskew, or intensity normalization.
        """
        #todo: Add M20 lidar post-processing before publishing the point cloud.
        points = np.asarray(raw.points, dtype=np.float32)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError(f"M20 lidar points must have shape (N, 3), got {points.shape}")

        intensities = raw.intensities
        if intensities is not None:
            intensities = np.asarray(intensities, dtype=np.float32)

        return PointCloud2.from_numpy(
            points,
            frame_id=raw.frame_id or self.config.lidar_frame_id,
            timestamp=raw.timestamp if raw.timestamp is not None else time.time(),
            intensities=intensities,
        )

    def _to_image(self, raw: M20CameraFrame) -> Image:
        """Convert SDK camera output to DimOS Image.

        Add image post-processing here: debayering, undistortion, resize,
        color conversion, exposure filtering, or timestamp repair.
        """
        #todo: Add M20 camera post-processing before publishing the image.
        frame = np.asarray(raw.image)
        if frame.ndim not in (2, 3):
            raise ValueError(f"M20 camera frame must be HxW or HxWxC, got {frame.shape}")
        if frame.dtype != np.uint8:
            frame = frame.astype(np.uint8)

        return Image.from_numpy(
            frame,
            format=raw.format or self.config.image_format,
            frame_id=raw.frame_id or self.config.camera_frame_id,
            ts=raw.timestamp if raw.timestamp is not None else time.time(),
        )

    @staticmethod
    def _sleep_remaining(period: float, started_at: float) -> None:
        if period <= 0:
            return
        elapsed = time.monotonic() - started_at
        remaining = period - elapsed
        if remaining > 0:
            time.sleep(remaining)

    @skill
    def observe(self) -> Image | None:
        """Return the latest M20 camera frame."""
        return self._latest_image


# Backwards-compatible name for early local experiments.
M20Connection = M20Sensor
