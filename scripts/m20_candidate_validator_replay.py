#!/usr/bin/env python3
"""Replay the physical candidate policy over saved shadow reports."""

from __future__ import annotations

import argparse
from collections import Counter
import csv
import json
import math
from pathlib import Path
from typing import Any

from dimos.mapping.occupancy.path_resampling import (
    PathPhysicalMetrics,
    select_physical_path_candidate,
)

DEFAULT_CLEARANCE_THRESHOLDS = (0.0, 0.01, 0.02, 0.025, 0.03, 0.05)
DEFAULT_UNKNOWN_THRESHOLDS = (0.0, 0.005, 0.01)


def _metrics(payload: dict[str, Any]) -> PathPhysicalMetrics:
    return PathPhysicalMetrics(
        path_length_m=float(payload["path_length_m"]),
        cumulative_turn_rad=float(payload["cumulative_turn_rad"]),
        min_clearance_m=(
            None if payload["min_clearance_m"] is None else float(payload["min_clearance_m"])
        ),
        p5_clearance_m=(
            None if payload["p5_clearance_m"] is None else float(payload["p5_clearance_m"])
        ),
        unknown_length_m=float(payload["unknown_length_m"]),
        unknown_ratio=float(payload["unknown_ratio"]),
        mean_cost=None if payload["mean_cost"] is None else float(payload["mean_cost"]),
        validation_reason=payload["validation_reason"],
    )


def _selected_metrics(shadow: dict[str, Any], alpha: float | None) -> PathPhysicalMetrics:
    if alpha is None:
        return _metrics(shadow["raw"])
    candidate = next(
        candidate
        for candidate in shadow["candidates"]
        if math.isclose(float(candidate["alpha"]), alpha)
    )
    return _metrics(candidate)


def replay_reports(
    report_paths: list[Path],
    clearance_thresholds: tuple[float, ...] = DEFAULT_CLEARANCE_THRESHOLDS,
    unknown_thresholds: tuple[float, ...] = DEFAULT_UNKNOWN_THRESHOLDS,
) -> dict[str, Any]:
    records: list[tuple[str, dict[str, Any]]] = []
    for path in report_paths:
        report = json.loads(path.read_text(encoding="utf-8"))
        records.extend(
            (path.name, case["shadow"])
            for case in report["cases"]
            if case.get("status") == "planned" and "shadow" in case
        )

    matrix: list[dict[str, Any]] = []
    for clearance_threshold in clearance_thresholds:
        for unknown_threshold in unknown_thresholds:
            selection_counts: Counter[str] = Counter()
            rejection_counts: Counter[str] = Counter()
            decision_differences = 0
            length_changes: list[float] = []
            turn_changes: list[float] = []
            violations = 0
            for _, shadow in records:
                raw = _metrics(shadow["raw"])
                candidates = [
                    (float(candidate["alpha"]), _metrics(candidate))
                    for candidate in shadow["candidates"]
                ]
                selected, decisions = select_physical_path_candidate(
                    raw,
                    candidates,
                    max_clearance_loss_m=clearance_threshold,
                    max_unknown_length_increase_m=unknown_threshold,
                )
                selection_counts["raw" if selected is None else str(selected)] += 1
                rejection_counts.update(
                    decision.rejection_reason
                    for decision in decisions
                    if decision.rejection_reason is not None
                )
                legacy_alpha = shadow.get("legacy_selected_alpha", shadow.get("selected_alpha"))
                decision_differences += selected != legacy_alpha
                selected_metrics = _selected_metrics(shadow, selected)
                length_changes.append(selected_metrics.path_length_m - raw.path_length_m)
                turn_changes.append(selected_metrics.cumulative_turn_rad - raw.cumulative_turn_rad)
                selected_decision = next(
                    (decision for decision in decisions if decision.alpha == selected), None
                )
                violations += selected_decision is not None and not selected_decision.accepted

            total = len(records)
            raw_count = selection_counts["raw"]
            matrix.append(
                {
                    "max_clearance_loss_m": clearance_threshold,
                    "max_unknown_length_increase_m": unknown_threshold,
                    "plans": total,
                    "selection_counts": dict(sorted(selection_counts.items())),
                    "raw_fallback_count": raw_count,
                    "raw_fallback_ratio": raw_count / total if total else 0.0,
                    "rejection_counts": dict(sorted(rejection_counts.items())),
                    "legacy_decision_differences": decision_differences,
                    "legacy_decision_difference_ratio": (
                        decision_differences / total if total else 0.0
                    ),
                    "mean_path_length_change_m": (sum(length_changes) / total if total else 0.0),
                    "mean_cumulative_turn_change_rad": (
                        sum(turn_changes) / total if total else 0.0
                    ),
                    "selected_policy_violations": violations,
                }
            )

    return {
        "inputs": [str(path) for path in report_paths],
        "successful_plans": len(records),
        "policy": {
            "hard_rejections": ["lethal_cell", "out_of_bounds"],
            "selection": "largest passing alpha, otherwise raw",
        },
        "matrix": matrix,
    }


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("reports", nargs="+", type=Path)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--output-csv", type=Path, required=True)
    return parser.parse_args()


def main() -> None:
    args = _arguments()
    replay = replay_reports(args.reports)
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(replay, indent=2), encoding="utf-8")

    rows = replay["matrix"]
    with args.output_csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=[
                "max_clearance_loss_m",
                "max_unknown_length_increase_m",
                "plans",
                "raw_fallback_count",
                "raw_fallback_ratio",
                "legacy_decision_differences",
                "legacy_decision_difference_ratio",
                "mean_path_length_change_m",
                "mean_cumulative_turn_change_rad",
                "selected_policy_violations",
                "selection_counts",
                "rejection_counts",
            ],
        )
        writer.writeheader()
        for row in rows:
            writer.writerow(
                {
                    **row,
                    "selection_counts": json.dumps(row["selection_counts"], sort_keys=True),
                    "rejection_counts": json.dumps(row["rejection_counts"], sort_keys=True),
                }
            )

    print(json.dumps({"successful_plans": replay["successful_plans"], "rows": len(rows)}))


if __name__ == "__main__":
    main()
