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


from dataclasses import asdict, dataclass
from itertools import pairwise
import json
import math
from time import perf_counter

import numpy as np
from scipy.ndimage import distance_transform_edt, uniform_filter1d

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues, OccupancyGrid
from dimos.msgs.nav_msgs.Path import Path
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import euler_to_quaternion

logger = setup_logger()

_PATH_SMOOTHING_TIMING_FIELDS = (
    "raw_validation_ms",
    "reference_costs_ms",
    "smoothing_loop_ms",
    "distance_transform_ms",
    "raw_resample_metrics_ms",
    "candidate_1_0_ms",
    "candidate_0_5_ms",
    "candidate_0_25_ms",
    "candidate_0_125_ms",
    "physical_policy_ms",
    "final_path_message_ms",
    "optimizer_total_ms",
    "path_publish_ms",
    "local_planner_handoff_ms",
)


def _initialize_smoothing_timing(
    timing: dict[str, float | int] | None,
    path: Path,
    costmap: OccupancyGrid,
) -> None:
    if timing is None:
        return
    timing.clear()
    timing.update({field: 0.0 for field in _PATH_SMOOTHING_TIMING_FIELDS})
    timing["raw_path_points"] = len(path.poses)
    timing["raw_path_length_m"] = sum(
        math.hypot(following.x - current.x, following.y - current.y)
        for current, following in pairwise(path.poses)
    )
    timing["costmap_width"] = costmap.width
    timing["costmap_height"] = costmap.height
    timing["smoothing_iterations"] = 0


def _record_elapsed(
    timing: dict[str, float | int] | None,
    field: str,
    started: float,
) -> float:
    elapsed_ms = (perf_counter() - started) * 1000
    if timing is not None:
        timing[field] = elapsed_ms
    return elapsed_ms


@dataclass(frozen=True)
class ConstrainedPathSmoothingConfig:
    spacing_m: float = 0.1
    max_iterations: int = 40
    data_weight: float = 0.02
    smoothness_weight: float = 0.45
    max_deviation_m: float = 0.1
    collision_sample_spacing_m: float = 0.05
    max_cost_increase: float = 2.0
    backtracking_factor: float = 0.5
    max_backtracking_steps: int = 3
    validator_shadow_enabled: bool = False
    physical_validator_shadow_enabled: bool = False
    physical_validator_authoritative_enabled: bool = False
    physical_validator_max_clearance_loss_m: float = 0.025
    physical_validator_max_unknown_length_increase_m: float = 0.0

    def __post_init__(self) -> None:
        if self.spacing_m <= 0 or self.collision_sample_spacing_m <= 0:
            raise ValueError("Path smoothing sample spacing must be positive")
        if self.max_iterations < 0 or self.max_deviation_m < 0:
            raise ValueError("Path smoothing limits must be non-negative")
        if not 0 <= self.data_weight <= 1:
            raise ValueError("data_weight must be between 0 and 1")
        if not 0 <= self.smoothness_weight <= 0.5:
            raise ValueError("smoothness_weight must be between 0 and 0.5")
        if self.max_cost_increase < 0:
            raise ValueError("max_cost_increase must be non-negative")
        if not 0 < self.backtracking_factor < 1:
            raise ValueError("backtracking_factor must be between 0 and 1")
        if self.max_backtracking_steps < 0:
            raise ValueError("max_backtracking_steps must be non-negative")
        if self.physical_validator_max_clearance_loss_m < 0:
            raise ValueError("physical validator clearance loss must be non-negative")
        if self.physical_validator_max_unknown_length_increase_m < 0:
            raise ValueError("physical validator unknown length increase must be non-negative")


@dataclass(frozen=True)
class PathPhysicalMetrics:
    path_length_m: float
    cumulative_turn_rad: float
    min_clearance_m: float | None
    p5_clearance_m: float | None
    unknown_length_m: float
    unknown_ratio: float
    mean_cost: float | None
    validation_reason: str | None


@dataclass(frozen=True)
class PhysicalCandidateDecision:
    alpha: float
    accepted: bool
    rejection_reason: str | None
    min_clearance_loss_m: float | None
    unknown_length_increase_m: float


def select_physical_path_candidate(
    raw: PathPhysicalMetrics,
    candidates: list[tuple[float, PathPhysicalMetrics]],
    *,
    max_clearance_loss_m: float,
    max_unknown_length_increase_m: float,
    epsilon: float = 1e-9,
) -> tuple[float | None, list[PhysicalCandidateDecision]]:
    """Select the largest physically valid smoothing alpha."""
    if max_clearance_loss_m < 0 or max_unknown_length_increase_m < 0:
        raise ValueError("physical validator thresholds must be non-negative")

    selected_alpha: float | None = None
    decisions: list[PhysicalCandidateDecision] = []
    for alpha, candidate in sorted(candidates, key=lambda item: item[0], reverse=True):
        clearance_loss = (
            raw.min_clearance_m - candidate.min_clearance_m
            if raw.min_clearance_m is not None and candidate.min_clearance_m is not None
            else None
        )
        unknown_increase = candidate.unknown_length_m - raw.unknown_length_m
        reason = candidate.validation_reason
        if (
            reason is None
            and clearance_loss is not None
            and clearance_loss > max_clearance_loss_m + epsilon
        ):
            reason = "clearance_loss"
        if reason is None and unknown_increase > max_unknown_length_increase_m + epsilon:
            reason = "unknown_length_increase"

        accepted = reason is None
        decisions.append(
            PhysicalCandidateDecision(
                alpha=alpha,
                accepted=accepted,
                rejection_reason=reason,
                min_clearance_loss_m=clearance_loss,
                unknown_length_increase_m=unknown_increase,
            )
        )
        if accepted and selected_alpha is None:
            selected_alpha = alpha

    return selected_alpha, decisions


def _lethal_clearance_grid(costmap: OccupancyGrid) -> np.ndarray | None:
    lethal = costmap.grid >= CostValues.OCCUPIED
    if not np.any(lethal):
        return None
    return distance_transform_edt(~lethal) * costmap.resolution


def _sample_path_geometry(
    points: np.ndarray,
    sample_spacing_m: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return endpoint samples plus midpoint samples and their represented lengths."""
    if len(points) == 0:
        empty = np.empty((0, 2), dtype=np.float64)
        return empty, empty, np.empty(0, dtype=np.float64)
    if len(points) == 1:
        return points.copy(), np.empty((0, 2)), np.empty(0)

    clearance_samples = [points[0]]
    midpoint_samples: list[np.ndarray] = []
    midpoint_lengths: list[float] = []
    for start, end in pairwise(points):
        segment = end - start
        length = float(np.linalg.norm(segment))
        if length <= 1e-10:
            continue
        sample_count = max(1, math.ceil(length / sample_spacing_m))
        represented_length = length / sample_count
        for sample_index in range(sample_count):
            start_ratio = sample_index / sample_count
            end_ratio = (sample_index + 1) / sample_count
            midpoint_samples.append(start + (start_ratio + end_ratio) * 0.5 * segment)
            midpoint_lengths.append(represented_length)
            clearance_samples.append(start + end_ratio * segment)

    return (
        np.asarray(clearance_samples, dtype=np.float64),
        np.asarray(midpoint_samples, dtype=np.float64).reshape(-1, 2),
        np.asarray(midpoint_lengths, dtype=np.float64),
    )


def _path_physical_metrics(
    points: np.ndarray,
    costmap: OccupancyGrid,
    sample_spacing_m: float,
    clearance_grid: np.ndarray | None = None,
) -> PathPhysicalMetrics:
    segments = np.diff(points, axis=0) if len(points) > 1 else np.empty((0, 2))
    segment_lengths = np.linalg.norm(segments, axis=1)
    path_length = float(np.sum(segment_lengths))

    nonzero_segments = segments[segment_lengths > 1e-10]
    if len(nonzero_segments) > 1:
        headings = np.arctan2(nonzero_segments[:, 1], nonzero_segments[:, 0])
        heading_deltas = np.diff(headings)
        heading_deltas = np.arctan2(np.sin(heading_deltas), np.cos(heading_deltas))
        cumulative_turn = float(np.sum(np.abs(heading_deltas)))
    else:
        cumulative_turn = 0.0

    clearance_samples, midpoint_samples, midpoint_lengths = _sample_path_geometry(
        points, sample_spacing_m
    )
    clearances: list[float] = []
    for point in clearance_samples:
        grid_point = costmap.world_to_grid((float(point[0]), float(point[1]), 0.0))
        grid_x = math.floor(grid_point.x)
        grid_y = math.floor(grid_point.y)
        if not (0 <= grid_x < costmap.width and 0 <= grid_y < costmap.height):
            clearances.append(0.0)
        elif clearance_grid is not None:
            clearances.append(float(clearance_grid[grid_y, grid_x]))

    unknown_length = 0.0
    for point, represented_length in zip(midpoint_samples, midpoint_lengths, strict=True):
        grid_point = costmap.world_to_grid((float(point[0]), float(point[1]), 0.0))
        grid_x = math.floor(grid_point.x)
        grid_y = math.floor(grid_point.y)
        if (
            0 <= grid_x < costmap.width
            and 0 <= grid_y < costmap.height
            and costmap.grid[grid_y, grid_x] == CostValues.UNKNOWN
        ):
            unknown_length += float(represented_length)

    mean_cost, validation_reason = _path_cost_validation(
        points,
        costmap,
        sample_spacing_m,
    )
    return PathPhysicalMetrics(
        path_length_m=path_length,
        cumulative_turn_rad=cumulative_turn,
        min_clearance_m=min(clearances) if clearances else None,
        p5_clearance_m=float(np.percentile(clearances, 5)) if clearances else None,
        unknown_length_m=unknown_length,
        unknown_ratio=unknown_length / path_length if path_length > 0 else 0.0,
        mean_cost=mean_cost,
        validation_reason=validation_reason,
    )


def _metrics_for_log(metrics: PathPhysicalMetrics) -> dict[str, float | str | None]:
    return {
        key: round(value, 4) if isinstance(value, float) else value
        for key, value in asdict(metrics).items()
    }


def _validator_metrics_enabled(config: ConstrainedPathSmoothingConfig) -> bool:
    return (
        config.validator_shadow_enabled
        or config.physical_validator_shadow_enabled
        or config.physical_validator_authoritative_enabled
    )


def _log_raw_only_validator_shadow(
    path: Path,
    costmap: OccupancyGrid,
    config: ConstrainedPathSmoothingConfig,
    *,
    reason: str,
    timing: dict[str, float | int] | None = None,
) -> None:
    if not _validator_metrics_enabled(config):
        return

    started = perf_counter()
    points = np.array([[pose.x, pose.y] for pose in path.poses], dtype=np.float64).reshape(-1, 2)
    clearance_started = perf_counter()
    clearance_grid = _lethal_clearance_grid(costmap)
    clearance_transform_ms = (perf_counter() - clearance_started) * 1000
    if timing is not None:
        timing["distance_transform_ms"] = clearance_transform_ms
    metrics_started = perf_counter()
    raw_metrics = _path_physical_metrics(
        points,
        costmap,
        config.collision_sample_spacing_m,
        clearance_grid,
    )
    candidate_evaluation_ms = (perf_counter() - metrics_started) * 1000
    if timing is not None:
        timing["raw_resample_metrics_ms"] = (perf_counter() - started) * 1000
    raw_valid = raw_metrics.validation_reason is None
    physical_enabled = (
        config.physical_validator_shadow_enabled or config.physical_validator_authoritative_enabled
    )
    shadow_report = {
        "legacy_selected_alpha": None,
        "physical_selected_alpha": None,
        "physical_decision_matches_legacy": raw_valid if physical_enabled else None,
        "physical_validator_evaluated": physical_enabled,
        "physical_max_clearance_loss_m": config.physical_validator_max_clearance_loss_m,
        "physical_max_unknown_length_increase_m": (
            config.physical_validator_max_unknown_length_increase_m
        ),
        "legacy_max_allowed_cost": None,
        "selected_alpha": None,
        "selected_path": "raw_resampled",
        "raw_baseline_valid": raw_valid,
        "raw_only_reason": reason,
        "raw": _metrics_for_log(raw_metrics),
        "candidates": [],
        "timing": {
            "clearance_transform_ms": round(clearance_transform_ms, 4),
            "candidate_evaluation_ms": round(candidate_evaluation_ms, 4),
            "validator_total_ms": round((perf_counter() - started) * 1000, 4),
        },
    }
    logger.info(
        "Candidate path validator shadow metrics.",
        shadow_report=json.dumps(shadow_report, separators=(",", ":")),
        selected_alpha=None,
        selected_path="raw_resampled",
    )


def _add_orientations_to_path(path: Path, goal_orientation: Quaternion) -> None:
    """Add orientations to path poses based on direction of movement.

    Args:
        path: Path with poses to add orientations to
        goal_orientation: Desired orientation for the final pose

    Returns:
        Path with orientations added to all poses
    """
    if not path.poses or len(path.poses) < 2:
        return

    # Calculate orientations for all poses except the last one
    for i in range(len(path.poses) - 1):
        current_pose = path.poses[i]
        next_pose = path.poses[i + 1]

        # Calculate direction to next point
        dx = next_pose.position.x - current_pose.position.x
        dy = next_pose.position.y - current_pose.position.y

        # Calculate yaw angle
        yaw = math.atan2(dy, dx)

        # Convert to quaternion (roll=0, pitch=0, yaw)
        orientation = euler_to_quaternion(Vector3(0, 0, yaw))
        current_pose.orientation = orientation

    # Set last pose orientation
    identity_quat = Quaternion(0, 0, 0, 1)
    if goal_orientation != identity_quat:
        # Use the provided goal orientation if it's not the identity
        path.poses[-1].orientation = goal_orientation
    elif len(path.poses) > 1:
        # Use the previous pose's orientation
        path.poses[-1].orientation = path.poses[-2].orientation
    else:
        # Single pose with identity goal orientation
        path.poses[-1].orientation = identity_quat


# TODO: replace goal_pose with just goal_orientation
def simple_resample_path(path: Path, goal_pose: Pose, spacing: float) -> Path:
    """Resample a path to have approximately uniform spacing between poses.

    Args:
        path: The original Path
        spacing: Desired distance between consecutive poses

    Returns:
        A new Path with resampled poses
    """
    if len(path) < 2 or spacing <= 0:
        return path

    resampled = []
    resampled.append(path.poses[0])

    accumulated_distance = 0.0

    for i in range(1, len(path.poses)):
        current = path.poses[i]
        prev = path.poses[i - 1]

        # Calculate segment distance
        dx = current.x - prev.x
        dy = current.y - prev.y
        segment_length = (dx**2 + dy**2) ** 0.5

        if segment_length < 1e-10:
            continue

        # Direction vector
        dir_x = dx / segment_length
        dir_y = dy / segment_length

        # Add points along this segment
        while accumulated_distance + segment_length >= spacing:
            # Distance along segment for next point
            dist_along = spacing - accumulated_distance
            if dist_along < 0:
                break

            # Create new pose
            new_x = prev.x + dir_x * dist_along
            new_y = prev.y + dir_y * dist_along
            new_pose = PoseStamped(
                frame_id=path.frame_id,
                position=[new_x, new_y, 0.0],
                orientation=prev.orientation,  # Keep same orientation
            )
            resampled.append(new_pose)

            # Update for next iteration
            accumulated_distance = 0
            segment_length -= dist_along
            prev = new_pose

        accumulated_distance += segment_length

    # Add last pose if not already there
    if len(path.poses) > 1:
        last = path.poses[-1]
        if not resampled or (resampled[-1].x != last.x or resampled[-1].y != last.y):
            resampled.append(last)

    ret = Path(frame_id=path.frame_id, poses=resampled)

    _add_orientations_to_path(ret, goal_pose.orientation)

    return ret


def _path_cost_validation(
    points: np.ndarray,
    costmap: OccupancyGrid,
    sample_spacing_m: float,
) -> tuple[float | None, str | None]:
    """Return mean traversable cost and a failure reason when invalid."""
    if len(points) == 1:
        grid_point = costmap.world_to_grid((float(points[0, 0]), float(points[0, 1]), 0.0))
        grid_x = math.floor(grid_point.x)
        grid_y = math.floor(grid_point.y)
        if not (0 <= grid_x < costmap.width and 0 <= grid_y < costmap.height):
            return None, "out_of_bounds"
        value = int(costmap.grid[grid_y, grid_x])
        if value >= CostValues.OCCUPIED:
            return None, "lethal_cell"
        return (80.0 if value == CostValues.UNKNOWN else max(0.0, float(value))), None

    values: list[float] = []
    for segment_index, (start, end) in enumerate(pairwise(points)):
        length = float(np.linalg.norm(end - start))
        sample_count = max(1, math.ceil(length / sample_spacing_m))
        first_sample = 0 if segment_index == 0 else 1
        for sample_index in range(first_sample, sample_count + 1):
            ratio = sample_index / sample_count
            point = start + ratio * (end - start)
            grid_point = costmap.world_to_grid((float(point[0]), float(point[1]), 0.0))
            grid_x = math.floor(grid_point.x)
            grid_y = math.floor(grid_point.y)
            if not (0 <= grid_x < costmap.width and 0 <= grid_y < costmap.height):
                return None, "out_of_bounds"

            value = int(costmap.grid[grid_y, grid_x])
            if value >= CostValues.OCCUPIED:
                return None, "lethal_cell"
            # Match min_cost_astar's default unknown penalty: 0.8 * 100.
            values.append(80.0 if value == CostValues.UNKNOWN else max(0.0, float(value)))

    return (float(np.mean(values)) if values else 0.0), None


def _effective_path_cost(
    points: np.ndarray,
    costmap: OccupancyGrid,
    sample_spacing_m: float,
) -> float | None:
    return _path_cost_validation(points, costmap, sample_spacing_m)[0]


def _path_from_xy(path: Path, points: np.ndarray) -> Path:
    return Path(
        frame_id=path.frame_id,
        poses=[
            PoseStamped(
                frame_id=path.frame_id,
                position=[float(point[0]), float(point[1]), 0.0],
                orientation=Quaternion(0, 0, 0, 1),
            )
            for point in points
        ],
    )


def _resample_xy_array(points: np.ndarray, spacing_m: float) -> np.ndarray:
    """Resample XY geometry without constructing ROS-style message objects."""
    if len(points) < 2 or spacing_m <= 0:
        return points.copy()

    # Preserve the legacy arithmetic order exactly: boundary-cell selection can
    # change when an otherwise negligible interpolation difference crosses a grid line.
    resampled = [(float(points[0, 0]), float(points[0, 1]))]
    accumulated_distance = 0.0
    for index in range(1, len(points)):
        current_x = float(points[index, 0])
        current_y = float(points[index, 1])
        previous_x = float(points[index - 1, 0])
        previous_y = float(points[index - 1, 1])
        dx = current_x - previous_x
        dy = current_y - previous_y
        segment_length = (dx**2 + dy**2) ** 0.5
        if segment_length < 1e-10:
            continue

        direction_x = dx / segment_length
        direction_y = dy / segment_length
        while accumulated_distance + segment_length >= spacing_m:
            distance_along = spacing_m - accumulated_distance
            if distance_along < 0:
                break
            previous_x += direction_x * distance_along
            previous_y += direction_y * distance_along
            resampled.append((previous_x, previous_y))
            accumulated_distance = 0.0
            segment_length -= distance_along

        accumulated_distance += segment_length

    final_point = (float(points[-1, 0]), float(points[-1, 1]))
    if resampled[-1] != final_point:
        resampled.append(final_point)
    return np.asarray(resampled, dtype=np.float64)


def _finalize_xy_path(
    source_path: Path,
    points: np.ndarray,
    goal_pose: Pose,
    timing: dict[str, float | int] | None,
) -> Path:
    started = perf_counter()
    result = _path_from_xy(source_path, points)
    _add_orientations_to_path(result, goal_pose.orientation)
    _record_elapsed(timing, "final_path_message_ms", started)
    return result


def _resample_xy(
    source_path: Path,
    points: np.ndarray,
    goal_pose: Pose,
    spacing_m: float,
) -> Path:
    return _finalize_xy_path(
        source_path,
        _resample_xy_array(points, spacing_m),
        goal_pose,
        None,
    )


def _select_backtracked_path(
    source_path: Path,
    original: np.ndarray,
    smoothed: np.ndarray,
    goal_pose: Pose,
    costmap: OccupancyGrid,
    config: ConstrainedPathSmoothingConfig,
    timing: dict[str, float | int] | None = None,
) -> Path:
    validator_started = perf_counter()
    physical_validator_enabled = (
        config.physical_validator_shadow_enabled or config.physical_validator_authoritative_enabled
    )
    metrics_enabled = config.validator_shadow_enabled or physical_validator_enabled
    raw_metrics_started = perf_counter()
    raw_resampled_points = _resample_xy_array(original, config.spacing_m)
    clearance_started = perf_counter()
    clearance_grid = _lethal_clearance_grid(costmap) if metrics_enabled else None
    clearance_transform_ms = (perf_counter() - clearance_started) * 1000
    if timing is not None:
        timing["distance_transform_ms"] = clearance_transform_ms
    candidate_evaluation_ms = 0.0
    if metrics_enabled:
        evaluation_started = perf_counter()
        raw_metrics: PathPhysicalMetrics | None = _path_physical_metrics(
            raw_resampled_points,
            costmap,
            config.collision_sample_spacing_m,
            clearance_grid,
        )
        candidate_evaluation_ms += (perf_counter() - evaluation_started) * 1000
        baseline_cost = raw_metrics.mean_cost
        baseline_failure_reason = raw_metrics.validation_reason
    else:
        raw_metrics = None
        baseline_cost, baseline_failure_reason = _path_cost_validation(
            raw_resampled_points,
            costmap,
            config.collision_sample_spacing_m,
        )
    _record_elapsed(timing, "raw_resample_metrics_ms", raw_metrics_started)
    if baseline_cost is None:
        if raw_metrics is not None:
            shadow_report = {
                "legacy_selected_alpha": None,
                "physical_selected_alpha": None,
                "physical_decision_matches_legacy": (True if physical_validator_enabled else None),
                "physical_validator_evaluated": physical_validator_enabled,
                "physical_max_clearance_loss_m": (config.physical_validator_max_clearance_loss_m),
                "physical_max_unknown_length_increase_m": (
                    config.physical_validator_max_unknown_length_increase_m
                ),
                "selected_alpha": None,
                "selected_path": "raw_resampled",
                "raw_baseline_valid": False,
                "raw": _metrics_for_log(raw_metrics),
                "candidates": [],
                "timing": {
                    "clearance_transform_ms": round(clearance_transform_ms, 4),
                    "candidate_evaluation_ms": round(candidate_evaluation_ms, 4),
                    "validator_total_ms": round((perf_counter() - validator_started) * 1000, 4),
                },
            }
            logger.info(
                "Candidate path validator shadow metrics.",
                shadow_report=json.dumps(shadow_report, separators=(",", ":")),
                selected_alpha=None,
                selected_path="raw_resampled",
            )
        logger.warning(
            "Raw-resampled baseline failed path validation; using raw-resampled A* path.",
            reason=baseline_failure_reason,
            raw_points=len(original),
            baseline_points=len(raw_resampled_points),
        )
        return _finalize_xy_path(source_path, raw_resampled_points, goal_pose, timing)

    max_allowed_cost = baseline_cost + config.max_cost_increase
    full_offset = smoothed - original
    fractions = [
        config.backtracking_factor**step for step in range(config.max_backtracking_steps + 1)
    ]
    rejected_fractions: list[float] = []
    selected_points: np.ndarray | None = None
    selected_alpha: float | None = None
    shadow_candidates: list[dict[str, object]] = []
    candidate_metrics_by_alpha: list[tuple[float, PathPhysicalMetrics]] = []
    candidate_points_by_alpha: dict[float, np.ndarray] = {}

    for fraction in fractions:
        candidate_started = perf_counter()
        blended = original + fraction * full_offset
        candidate_points = _resample_xy_array(blended, config.spacing_m)
        if metrics_enabled:
            evaluation_started = perf_counter()
            candidate_metrics = _path_physical_metrics(
                candidate_points,
                costmap,
                config.collision_sample_spacing_m,
                clearance_grid,
            )
            candidate_evaluation_ms += (perf_counter() - evaluation_started) * 1000
            candidate_cost = candidate_metrics.mean_cost
            failure_reason = candidate_metrics.validation_reason
            candidate_metrics_by_alpha.append((fraction, candidate_metrics))
            candidate_points_by_alpha[fraction] = candidate_points
        else:
            candidate_metrics = None
            candidate_cost, failure_reason = _path_cost_validation(
                candidate_points,
                costmap,
                config.collision_sample_spacing_m,
            )
        rejection_reason = failure_reason
        if candidate_cost is not None and candidate_cost > max_allowed_cost:
            rejection_reason = "cost_increase"

        if candidate_metrics is not None:
            shadow_candidates.append(
                {
                    "alpha": round(fraction, 4),
                    "legacy_gate_passed": rejection_reason is None,
                    "legacy_rejection_reason": rejection_reason,
                    **_metrics_for_log(candidate_metrics),
                }
            )

        if timing is not None:
            timing[f"candidate_{str(fraction).replace('.', '_')}_ms"] = (
                perf_counter() - candidate_started
            ) * 1000

        if rejection_reason is None:
            assert candidate_cost is not None
            if selected_points is None:
                selected_points = candidate_points
                selected_alpha = fraction
                logger.info(
                    "Constrained path smoothing accepted.",
                    selected_fraction=round(fraction, 4),
                    baseline_cost=round(baseline_cost, 3),
                    candidate_cost=round(candidate_cost, 3),
                    max_allowed_cost=round(max_allowed_cost, 3),
                    rejected_fractions=rejected_fractions,
                    raw_points=len(original),
                    candidate_points=len(candidate_points),
                    max_deviation_m=round(
                        float(np.max(np.linalg.norm(blended - original, axis=1))),
                        3,
                    ),
                )
                if not metrics_enabled:
                    return _finalize_xy_path(source_path, candidate_points, goal_pose, timing)
            continue

        if selected_points is None:
            rejected_fractions.append(round(fraction, 4))
            logger.info(
                "Constrained path smoothing fraction rejected.",
                fraction=round(fraction, 4),
                reason=rejection_reason,
                baseline_cost=round(baseline_cost, 3),
                candidate_cost=None if candidate_cost is None else round(candidate_cost, 3),
                max_allowed_cost=round(max_allowed_cost, 3),
            )

    physical_selected_alpha: float | None = None
    if raw_metrics is not None and physical_validator_enabled:
        physical_policy_started = perf_counter()
        physical_selected_alpha, physical_decisions = select_physical_path_candidate(
            raw_metrics,
            candidate_metrics_by_alpha,
            max_clearance_loss_m=config.physical_validator_max_clearance_loss_m,
            max_unknown_length_increase_m=(config.physical_validator_max_unknown_length_increase_m),
        )
        _record_elapsed(timing, "physical_policy_ms", physical_policy_started)
        for candidate, decision in zip(shadow_candidates, physical_decisions, strict=True):
            candidate.update(
                {
                    "physical_gate_passed": decision.accepted,
                    "physical_rejection_reason": decision.rejection_reason,
                    "min_clearance_loss_m": (
                        None
                        if decision.min_clearance_loss_m is None
                        else round(decision.min_clearance_loss_m, 4)
                    ),
                    "unknown_length_increase_m": round(decision.unknown_length_increase_m, 4),
                }
            )

    if raw_metrics is not None:
        decision_matches = (
            physical_selected_alpha == selected_alpha if physical_validator_enabled else None
        )
        shadow_report = {
            "legacy_selected_alpha": selected_alpha,
            "physical_selected_alpha": physical_selected_alpha,
            "physical_decision_matches_legacy": decision_matches,
            "physical_validator_evaluated": physical_validator_enabled,
            "physical_max_clearance_loss_m": config.physical_validator_max_clearance_loss_m,
            "physical_max_unknown_length_increase_m": (
                config.physical_validator_max_unknown_length_increase_m
            ),
            "legacy_max_allowed_cost": round(max_allowed_cost, 4),
            "selected_alpha": selected_alpha,
            "selected_path": "raw_resampled" if selected_points is None else "candidate",
            "raw_baseline_valid": True,
            "raw": _metrics_for_log(raw_metrics),
            "candidates": shadow_candidates,
            "timing": {
                "clearance_transform_ms": round(clearance_transform_ms, 4),
                "candidate_evaluation_ms": round(candidate_evaluation_ms, 4),
                "validator_total_ms": round((perf_counter() - validator_started) * 1000, 4),
            },
        }
        logger.info(
            "Candidate path validator shadow metrics.",
            shadow_report=json.dumps(shadow_report, separators=(",", ":")),
            selected_alpha=None if selected_alpha is None else round(selected_alpha, 4),
            selected_path="raw_resampled" if selected_points is None else "candidate",
            legacy_max_allowed_cost=round(max_allowed_cost, 4),
        )

    if config.physical_validator_authoritative_enabled:
        if physical_selected_alpha is None:
            return _finalize_xy_path(source_path, raw_resampled_points, goal_pose, timing)
        return _finalize_xy_path(
            source_path,
            candidate_points_by_alpha[physical_selected_alpha],
            goal_pose,
            timing,
        )

    if selected_points is not None:
        return _finalize_xy_path(source_path, selected_points, goal_pose, timing)

    logger.warning(
        "All constrained smoothing fractions failed; using raw-resampled A* path.",
        rejected_fractions=rejected_fractions,
        baseline_cost=round(baseline_cost, 3),
        max_allowed_cost=round(max_allowed_cost, 3),
        raw_points=len(original),
    )
    return _finalize_xy_path(source_path, raw_resampled_points, goal_pose, timing)


def constrained_smooth_resample_path(
    path: Path,
    goal_pose: Pose,
    costmap: OccupancyGrid,
    config: ConstrainedPathSmoothingConfig,
    timing: dict[str, float | int] | None = None,
) -> Path:
    """Locally smooth a grid path while preserving its costmap corridor."""
    optimizer_started = perf_counter()
    _initialize_smoothing_timing(timing, path, costmap)
    try:
        if len(path) < 3 or config.max_iterations == 0 or config.max_deviation_m == 0:
            raw_message_started = perf_counter()
            raw_resampled = simple_resample_path(path, goal_pose, config.spacing_m)
            _record_elapsed(timing, "final_path_message_ms", raw_message_started)
            _log_raw_only_validator_shadow(
                raw_resampled,
                costmap,
                config,
                reason="smoothing_not_applicable",
                timing=timing,
            )
            return raw_resampled

        original = np.array([[pose.x, pose.y] for pose in path.poses], dtype=np.float64)
        duplicate = np.linalg.norm(np.diff(original, axis=0), axis=1) <= 1e-10
        original = original[np.concatenate(([True], ~duplicate))]
        if len(original) < 3:
            raw_message_started = perf_counter()
            raw_resampled = simple_resample_path(path, goal_pose, config.spacing_m)
            _record_elapsed(timing, "final_path_message_ms", raw_message_started)
            _log_raw_only_validator_shadow(
                raw_resampled,
                costmap,
                config,
                reason="duplicate_points_removed",
                timing=timing,
            )
            return raw_resampled

        raw_validation_started = perf_counter()
        raw_cost, raw_failure_reason = _path_cost_validation(
            original,
            costmap,
            config.collision_sample_spacing_m,
        )
        _record_elapsed(timing, "raw_validation_ms", raw_validation_started)
        if raw_cost is None:
            raw_message_started = perf_counter()
            raw_resampled = simple_resample_path(path, goal_pose, config.spacing_m)
            _record_elapsed(timing, "final_path_message_ms", raw_message_started)
            _log_raw_only_validator_shadow(
                raw_resampled,
                costmap,
                config,
                reason="raw_astar_validation_failed",
                timing=timing,
            )
            logger.warning(
                "Raw A* path failed constrained-smoothing validation; skipping smoothing.",
                reason=raw_failure_reason,
                raw_points=len(original),
            )
            return raw_resampled

        smoothed = original.copy()
        reference_costs_started = perf_counter()
        reference_costs = [
            _effective_path_cost(
                original[index - 1 : index + 2],
                costmap,
                config.collision_sample_spacing_m,
            )
            for index in range(1, len(original) - 1)
        ]
        _record_elapsed(timing, "reference_costs_ms", reference_costs_started)
        smoothing_started = perf_counter()
        for iteration in range(config.max_iterations):
            if timing is not None:
                timing["smoothing_iterations"] = iteration + 1
            max_change = 0.0
            for index in range(1, len(smoothed) - 1):
                current = smoothed[index]
                candidate = current + config.data_weight * (original[index] - current)
                candidate += config.smoothness_weight * (
                    smoothed[index - 1] + smoothed[index + 1] - 2 * current
                )

                offset = candidate - original[index]
                offset_length = float(np.linalg.norm(offset))
                if offset_length > config.max_deviation_m:
                    candidate = original[index] + offset * (config.max_deviation_m / offset_length)

                candidate_points = np.vstack((smoothed[index - 1], candidate, smoothed[index + 1]))
                reference_cost = reference_costs[index - 1]
                candidate_cost = _effective_path_cost(
                    candidate_points,
                    costmap,
                    config.collision_sample_spacing_m,
                )
                if (
                    reference_cost is None
                    or candidate_cost is None
                    or candidate_cost > reference_cost + config.max_cost_increase
                ):
                    continue

                change = float(np.linalg.norm(candidate - current))
                smoothed[index] = candidate
                max_change = max(max_change, change)

            if max_change < 1e-4:
                break
        _record_elapsed(timing, "smoothing_loop_ms", smoothing_started)

        return _select_backtracked_path(
            path,
            original,
            smoothed,
            goal_pose,
            costmap,
            config,
            timing,
        )
    finally:
        _record_elapsed(timing, "optimizer_total_ms", optimizer_started)


def smooth_resample_path(
    path: Path, goal_pose: Pose, spacing: float, smoothing_window: int = 100
) -> Path:
    """Resample a path with smoothing to reduce jagged corners and abrupt turns.

    This produces smoother paths than simple_resample_path by:
    - First upsampling the path to have many points
    - Applying a moving average filter to smooth the coordinates
    - Resampling at the desired spacing
    - Keeping start and end points fixed

    Args:
        path: The original Path
        goal_pose: Goal pose with desired final orientation
        spacing: Desired approximate distance between consecutive poses
        smoothing_window: Size of the smoothing window (larger = smoother)

    Returns:
        A new Path with smoothly resampled poses
    """

    if len(path.poses) == 1:
        p = path.poses[0].position
        o = goal_pose.orientation
        new_pose = PoseStamped(
            frame_id=path.frame_id,
            position=[p.x, p.y, p.z],
            orientation=[o.x, o.y, o.z, o.w],
        )
        return Path(frame_id=path.frame_id, poses=[new_pose])

    if len(path) < 2 or spacing <= 0:
        return path

    # Extract x, y coordinates from path
    xs = np.array([p.x for p in path.poses])
    ys = np.array([p.y for p in path.poses])

    # Remove duplicate consecutive points
    diffs = np.sqrt(np.diff(xs) ** 2 + np.diff(ys) ** 2)
    valid_mask = np.concatenate([[True], diffs > 1e-10])
    xs = xs[valid_mask]
    ys = ys[valid_mask]

    if len(xs) < 2:
        return path

    # Calculate total path length
    dx = np.diff(xs)
    dy = np.diff(ys)
    segment_lengths = np.sqrt(dx**2 + dy**2)
    total_length = np.sum(segment_lengths)

    if total_length < spacing:
        return path

    # Upsample: create many points along the original path using linear interpolation
    # This gives us enough points for effective smoothing
    upsample_factor = 10
    num_upsampled = max(len(xs) * upsample_factor, 100)

    arc_length = np.concatenate([[0], np.cumsum(segment_lengths)])
    upsample_distances = np.linspace(0, total_length, num_upsampled)

    # Linear interpolation along arc length
    xs_upsampled = np.interp(upsample_distances, arc_length, xs)
    ys_upsampled = np.interp(upsample_distances, arc_length, ys)

    # Apply moving average smoothing
    # Use 'nearest' mode to avoid shrinking at boundaries
    window = min(smoothing_window, len(xs_upsampled) // 3)
    if window >= 3:
        xs_smooth = uniform_filter1d(xs_upsampled, size=window, mode="nearest")
        ys_smooth = uniform_filter1d(ys_upsampled, size=window, mode="nearest")
    else:
        xs_smooth = xs_upsampled
        ys_smooth = ys_upsampled

    # Keep start and end points exactly as original
    xs_smooth[0] = xs[0]
    ys_smooth[0] = ys[0]
    xs_smooth[-1] = xs[-1]
    ys_smooth[-1] = ys[-1]

    # Recalculate arc length on smoothed path
    dx_smooth = np.diff(xs_smooth)
    dy_smooth = np.diff(ys_smooth)
    segment_lengths_smooth = np.sqrt(dx_smooth**2 + dy_smooth**2)
    arc_length_smooth = np.concatenate([[0], np.cumsum(segment_lengths_smooth)])
    total_length_smooth = arc_length_smooth[-1]

    # Resample at desired spacing
    num_samples = max(2, int(np.ceil(total_length_smooth / spacing)) + 1)
    sample_distances = np.linspace(0, total_length_smooth, num_samples)

    # Interpolate to get final points
    sampled_x = np.interp(sample_distances, arc_length_smooth, xs_smooth)
    sampled_y = np.interp(sample_distances, arc_length_smooth, ys_smooth)

    # Create resampled poses
    resampled = []
    for i in range(len(sampled_x)):
        new_pose = PoseStamped(
            frame_id=path.frame_id,
            position=[float(sampled_x[i]), float(sampled_y[i]), 0.0],
            orientation=Quaternion(0, 0, 0, 1),
        )
        resampled.append(new_pose)

    ret = Path(frame_id=path.frame_id, poses=resampled)

    _add_orientations_to_path(ret, goal_pose.orientation)

    return ret
