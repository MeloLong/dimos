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

from dataclasses import replace
from unittest.mock import patch

import numpy as np
import pytest

from dimos.mapping.occupancy.gradient import gradient
from dimos.mapping.occupancy.path_resampling import (
    ConstrainedPathSmoothingConfig,
    _lethal_clearance_grid,
    _path_physical_metrics,
    constrained_smooth_resample_path,
    simple_resample_path,
    smooth_resample_path,
)
from dimos.mapping.occupancy.visualize_path import visualize_path
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.Image import Image
from dimos.navigation.replanning_a_star.min_cost_astar import min_cost_astar
from dimos.utils.data import get_data


@pytest.fixture
def costmap() -> OccupancyGrid:
    return gradient(OccupancyGrid(np.load(get_data("occupancy_simple.npy"))), max_distance=1.5)


@pytest.mark.parametrize("method", ["simple", "smooth"])
def test_resample_path(costmap, method) -> None:
    start = Vector3(4.0, 2.0, 0)
    goal_pose = Pose(6.15, 10.0, 0, 0, 0, 0, 1)
    expected = Image.from_file(get_data(f"resample_path_{method}.png"))
    path = min_cost_astar(costmap, goal_pose.position, start, use_cpp=False)

    match method:
        case "simple":
            resampled = simple_resample_path(path, goal_pose, 0.1)
        case "smooth":
            resampled = smooth_resample_path(path, goal_pose, 0.1)
        case _:
            raise ValueError(f"Unknown resampling method: {method}")

    actual = visualize_path(costmap, resampled, 0.2, 0.4)
    np.testing.assert_array_equal(actual.data, expected.data)


def test_path_physical_metrics_use_distance_weighted_unknown_exposure() -> None:
    grid = np.zeros((8, 8), dtype=np.int8)
    grid[4, :] = 100
    grid[1, 3:5] = -1
    costmap = OccupancyGrid(grid=grid, resolution=1.0)
    points = np.array([[1.5, 1.5], [5.5, 1.5]], dtype=np.float64)

    metrics = _path_physical_metrics(
        points,
        costmap,
        sample_spacing_m=1.0,
        clearance_grid=_lethal_clearance_grid(costmap),
    )

    assert metrics.path_length_m == pytest.approx(4.0)
    assert metrics.cumulative_turn_rad == pytest.approx(0.0)
    assert metrics.min_clearance_m == pytest.approx(3.0)
    assert metrics.p5_clearance_m == pytest.approx(3.0)
    assert metrics.unknown_length_m == pytest.approx(2.0)
    assert metrics.unknown_ratio == pytest.approx(0.5)
    assert metrics.validation_reason is None


@pytest.mark.parametrize(
    ("points", "expected_reason"),
    [
        (np.array([[1.5, 1.5], [1.5, 5.5]]), "lethal_cell"),
        (np.array([[-0.5, 1.5], [1.5, 1.5]]), "out_of_bounds"),
    ],
)
def test_path_physical_metrics_record_hard_failure_reason(points, expected_reason) -> None:
    grid = np.zeros((8, 8), dtype=np.int8)
    grid[4, :] = 100
    costmap = OccupancyGrid(grid=grid, resolution=1.0)

    metrics = _path_physical_metrics(points, costmap, sample_spacing_m=0.5)

    assert metrics.validation_reason == expected_reason


def test_validator_shadow_records_all_alphas_without_changing_path() -> None:
    costmap = OccupancyGrid(grid=np.zeros((80, 80), dtype=np.int8), resolution=0.1)
    path = Path(
        frame_id="map",
        poses=[
            PoseStamped(frame_id="map", position=[x, y, 0.0])
            for x, y in [(1.0, 1.0), (2.0, 1.3), (3.0, 0.8), (4.0, 1.3), (5.0, 1.0)]
        ],
    )
    goal = Pose(position=[5.0, 1.0, 0.0])
    config = ConstrainedPathSmoothingConfig(max_iterations=10)

    legacy_path = constrained_smooth_resample_path(path, goal, costmap, config)
    with patch("dimos.mapping.occupancy.path_resampling.logger.info") as log_info:
        shadow_path = constrained_smooth_resample_path(
            path,
            goal,
            costmap,
            replace(config, validator_shadow_enabled=True),
        )

    legacy_points = np.array([[pose.x, pose.y] for pose in legacy_path.poses])
    shadow_points = np.array([[pose.x, pose.y] for pose in shadow_path.poses])
    np.testing.assert_allclose(shadow_points, legacy_points)

    shadow_call = next(
        call
        for call in log_info.call_args_list
        if call.args[0] == "Candidate path validator shadow metrics."
    )
    assert shadow_call.kwargs["selected_alpha"] == 1.0
    assert len(shadow_call.kwargs["candidates"]) == 4
    assert shadow_call.kwargs["raw"]["path_length_m"] > 0.0
