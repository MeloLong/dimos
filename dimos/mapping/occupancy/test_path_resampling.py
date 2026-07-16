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
import json
from unittest.mock import patch

import numpy as np
import pytest

from dimos.mapping.occupancy.gradient import gradient
from dimos.mapping.occupancy.path_resampling import (
    ConstrainedPathSmoothingConfig,
    PathPhysicalMetrics,
    _lethal_clearance_grid,
    _path_physical_metrics,
    constrained_smooth_resample_path,
    select_physical_path_candidate,
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
        (np.array([[-0.5, 1.5]]), "out_of_bounds"),
    ],
)
def test_path_physical_metrics_record_hard_failure_reason(points, expected_reason) -> None:
    grid = np.zeros((8, 8), dtype=np.int8)
    grid[4, :] = 100
    costmap = OccupancyGrid(grid=grid, resolution=1.0)

    metrics = _path_physical_metrics(points, costmap, sample_spacing_m=0.5)

    assert metrics.validation_reason == expected_reason


def _physical_metrics(
    *,
    clearance: float | None = 1.0,
    unknown_length: float = 0.0,
    validation_reason: str | None = None,
) -> PathPhysicalMetrics:
    return PathPhysicalMetrics(
        path_length_m=4.0,
        cumulative_turn_rad=1.0,
        min_clearance_m=clearance,
        p5_clearance_m=clearance,
        unknown_length_m=unknown_length,
        unknown_ratio=unknown_length / 4.0,
        mean_cost=None if validation_reason else 1.0,
        validation_reason=validation_reason,
    )


@pytest.mark.parametrize(
    ("loss", "accepted"),
    [(0.024999, True), (0.025, True), (0.025001, False)],
)
def test_physical_validator_clearance_threshold_boundary(loss, accepted) -> None:
    selected, decisions = select_physical_path_candidate(
        _physical_metrics(),
        [(1.0, _physical_metrics(clearance=1.0 - loss))],
        max_clearance_loss_m=0.025,
        max_unknown_length_increase_m=0.0,
    )

    assert (selected == 1.0) is accepted
    assert decisions[0].accepted is accepted
    assert decisions[0].rejection_reason == (None if accepted else "clearance_loss")


@pytest.mark.parametrize(
    ("increase", "accepted"),
    [(-0.01, True), (0.0, True), (0.000001, False)],
)
def test_physical_validator_unknown_exposure_boundary(increase, accepted) -> None:
    selected, decisions = select_physical_path_candidate(
        _physical_metrics(unknown_length=0.2),
        [(1.0, _physical_metrics(unknown_length=0.2 + increase))],
        max_clearance_loss_m=0.025,
        max_unknown_length_increase_m=0.0,
    )

    assert (selected == 1.0) is accepted
    assert decisions[0].accepted is accepted
    assert decisions[0].rejection_reason == (None if accepted else "unknown_length_increase")


@pytest.mark.parametrize("reason", ["lethal_cell", "out_of_bounds"])
def test_physical_validator_always_rejects_hard_failures(reason) -> None:
    selected, decisions = select_physical_path_candidate(
        _physical_metrics(),
        [(1.0, _physical_metrics(validation_reason=reason))],
        max_clearance_loss_m=10.0,
        max_unknown_length_increase_m=10.0,
    )

    assert selected is None
    assert decisions[0].rejection_reason == reason


def test_physical_validator_selects_largest_passing_alpha() -> None:
    selected, decisions = select_physical_path_candidate(
        _physical_metrics(),
        [
            (0.5, _physical_metrics(clearance=0.98)),
            (0.25, _physical_metrics(clearance=0.99)),
            (1.0, _physical_metrics(clearance=0.9)),
        ],
        max_clearance_loss_m=0.025,
        max_unknown_length_increase_m=0.0,
    )

    assert selected == 0.5
    assert [decision.accepted for decision in decisions] == [False, True, True]


def test_physical_validator_falls_back_to_raw_when_all_candidates_fail() -> None:
    selected, decisions = select_physical_path_candidate(
        _physical_metrics(clearance=None),
        [
            (1.0, _physical_metrics(clearance=None, unknown_length=0.1)),
            (0.5, _physical_metrics(clearance=None, validation_reason="lethal_cell")),
        ],
        max_clearance_loss_m=0.0,
        max_unknown_length_increase_m=0.0,
    )

    assert selected is None
    assert [decision.rejection_reason for decision in decisions] == [
        "unknown_length_increase",
        "lethal_cell",
    ]


def test_physical_validator_accepts_empty_metric_domain_without_hard_failure() -> None:
    selected, decisions = select_physical_path_candidate(
        _physical_metrics(clearance=None),
        [(1.0, _physical_metrics(clearance=None))],
        max_clearance_loss_m=0.0,
        max_unknown_length_increase_m=0.0,
    )

    assert selected == 1.0
    assert decisions[0].accepted


def test_raw_invalid_path_emits_shadow_record_before_smoothing_skip() -> None:
    grid = np.zeros((80, 80), dtype=np.int8)
    grid[20, :] = 100
    costmap = OccupancyGrid(grid=grid, resolution=0.1)
    path = Path(
        frame_id="map",
        poses=[PoseStamped(frame_id="map", position=[1.0, y, 0.0]) for y in (1.0, 2.0, 3.0)],
    )
    goal = Pose(position=[1.0, 3.0, 0.0])
    config = ConstrainedPathSmoothingConfig(
        validator_shadow_enabled=True,
        physical_validator_shadow_enabled=True,
    )

    with patch("dimos.mapping.occupancy.path_resampling.logger.info") as log_info:
        result = constrained_smooth_resample_path(path, goal, costmap, config)

    assert result.poses
    shadow_call = next(
        call
        for call in log_info.call_args_list
        if call.args[0] == "Candidate path validator shadow metrics."
    )
    report = json.loads(shadow_call.kwargs["shadow_report"])
    assert report["raw_baseline_valid"] is False
    assert report["raw_only_reason"] == "raw_astar_validation_failed"
    assert report["raw"]["validation_reason"] == "lethal_cell"
    assert report["candidates"] == []


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
            replace(
                config,
                validator_shadow_enabled=True,
                physical_validator_shadow_enabled=True,
            ),
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
    report = json.loads(shadow_call.kwargs["shadow_report"])
    assert len(report["candidates"]) == 4
    assert report["raw"]["path_length_m"] > 0.0
    assert report["legacy_selected_alpha"] == 1.0
    assert report["physical_selected_alpha"] == 1.0
    assert report["physical_decision_matches_legacy"] is True
    assert all("physical_gate_passed" in candidate for candidate in report["candidates"])
    assert config.physical_validator_authoritative_enabled is False
