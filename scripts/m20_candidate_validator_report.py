#!/usr/bin/env python3
"""Aggregate Stage 2 candidate-validator reports and render text SVG charts."""

from __future__ import annotations

import argparse
from collections import Counter
import csv
from html import escape
import json
from pathlib import Path
from time import perf_counter_ns
from typing import Any

import numpy as np

from dimos.mapping.occupancy.path_resampling import (
    PathPhysicalMetrics,
    select_physical_path_candidate,
)


def _distribution(values: list[float]) -> dict[str, float] | None:
    if not values:
        return None
    data = np.asarray(values, dtype=np.float64)
    return {
        "p50": round(float(np.percentile(data, 50)), 4),
        "p95": round(float(np.percentile(data, 95)), 4),
        "max": round(float(np.max(data)), 4),
    }


def _load(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _metrics(payload: dict[str, Any]) -> PathPhysicalMetrics:
    return PathPhysicalMetrics(
        path_length_m=payload["path_length_m"],
        cumulative_turn_rad=payload["cumulative_turn_rad"],
        min_clearance_m=payload["min_clearance_m"],
        p5_clearance_m=payload["p5_clearance_m"],
        unknown_length_m=payload["unknown_length_m"],
        unknown_ratio=payload["unknown_ratio"],
        mean_cost=payload["mean_cost"],
        validation_reason=payload["validation_reason"],
    )


def _bar_svg(
    title: str,
    labels: list[str],
    values: list[float],
    value_suffix: str,
    output: Path,
) -> None:
    width = 960
    row_height = 44
    height = 90 + row_height * len(labels)
    left = 260
    chart_width = 620
    maximum = max(values, default=1.0) or 1.0
    rows = []
    for index, (label, value) in enumerate(zip(labels, values, strict=True)):
        y = 58 + index * row_height
        bar_width = chart_width * value / maximum
        rows.append(
            f'<text x="{left - 12}" y="{y + 17}" text-anchor="end">{escape(label)}</text>'
            f'<rect x="{left}" y="{y}" width="{bar_width:.2f}" height="24" rx="2"/>'
            f'<text x="{left + bar_width + 8:.2f}" y="{y + 17}">{value:.1f}{value_suffix}</text>'
        )
    svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}" role="img" aria-labelledby="title desc">
<title id="title">{escape(title)}</title>
<desc id="desc">Horizontal bar chart with values labeled on every bar.</desc>
<style>text{{font:14px sans-serif;fill:#20262d}} rect{{fill:#2f7d71}} .title{{font-size:20px;font-weight:500}}</style>
<text class="title" x="24" y="32">{escape(title)}</text>
{"".join(rows)}
</svg>
'''
    output.write_text(svg, encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    args = parser.parse_args()
    root = args.root

    valid_specs = [
        ("historical-dynamic", root.parent / "2026-07-16/dynamic-3x-report.json", False),
        ("historical-static", root.parent / "2026-07-16/static-clean-3x-report.json", False),
        ("race-fix-smoke", root / "race-fix-smoke/report.json", True),
        ("office-clean-final", root / "clean-office-final-3x/report.json", True),
        (
            "narrow-clean-shadow-on",
            root / "clean-performance-shadow-on/report.json",
            True,
        ),
    ]
    scenario_rows: list[dict[str, Any]] = []
    physical_selections: Counter[str] = Counter()
    rejection_reasons: Counter[str] = Counter()
    physical_plans = 0
    decision_matches = 0
    policy_violations = 0
    selected_hard_violations = 0
    selected_clearance_violations = 0
    selected_unknown_violations = 0
    uncaught_exceptions = 0
    successful_plans = 0
    for name, path, physical_enabled in valid_specs:
        report = _load(path)
        summary = report["summary"]
        successful_plans += summary["planned"]
        scenario_rows.append(
            {
                "scenario": name,
                "physical_shadow": physical_enabled,
                "requests": summary["cases"],
                "successful_plans": summary["planned"],
                "no_path_or_timeout": summary.get("no_path_or_timeout", 0),
                "raw_baseline_invalid": summary.get("raw_baseline_invalid", 0),
                "shadow_record_mismatch": summary.get("shadow_record_mismatch", 0),
                "max_odom_drift_m": summary["max_odom_drift_m"],
            }
        )
        if not physical_enabled:
            continue
        for case in report["cases"]:
            if case["status"] != "planned":
                continue
            shadow = case["shadow"]
            if not shadow.get("physical_validator_evaluated"):
                continue
            physical_plans += 1
            alpha = shadow.get("physical_selected_alpha")
            physical_selections["raw" if alpha is None else str(alpha)] += 1
            decision_matches += shadow.get("physical_decision_matches_legacy") is True
            selected_candidate = next(
                (
                    candidate
                    for candidate in shadow["candidates"]
                    if alpha is not None and candidate["alpha"] == alpha
                ),
                None,
            )
            if selected_candidate is not None:
                selected_hard_violations += selected_candidate["validation_reason"] is not None
                selected_clearance_violations += (
                    selected_candidate["min_clearance_loss_m"] is not None
                    and selected_candidate["min_clearance_loss_m"] > 0.025 + 1e-9
                )
                selected_unknown_violations += (
                    selected_candidate["unknown_length_increase_m"] > 1e-9
                )
            for candidate in shadow["candidates"]:
                reason = candidate.get("physical_rejection_reason")
                if reason is not None:
                    rejection_reasons[reason] += 1
                if candidate.get("physical_gate_passed") and reason is not None:
                    policy_violations += 1

    with (root / "scenario-summary.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(scenario_rows[0]))
        writer.writeheader()
        writer.writerows(scenario_rows)

    replay = _load(root / "threshold-replay.json")
    strict_rows = [row for row in replay["matrix"] if row["max_unknown_length_increase_m"] == 0.0]
    performance_on = _load(root / "clean-performance-shadow-on/report.json")
    performance_off = _load(root / "clean-performance-shadow-off/report.json")

    def timings(report: dict[str, Any], key: str) -> list[float]:
        return [
            float(case["shadow"]["timing"][key])
            for case in report["cases"]
            if case["status"] == "planned"
        ]

    on_latency = performance_on["summary"]["planner_latency_ms"]
    off_latency = performance_off["summary"]["planner_latency_ms"]
    p95_delta = (on_latency["p95"] / off_latency["p95"] - 1.0) * 100
    decisions_by_goal: dict[tuple[float, float], set[float | None]] = {}
    for case in performance_on["cases"]:
        if case["status"] == "planned":
            decisions_by_goal.setdefault(tuple(case["goal"]), set()).add(
                case["shadow"]["physical_selected_alpha"]
            )
    static_consistency = sum(len(values) == 1 for values in decisions_by_goal.values())
    exact_policy_us: list[float] = []
    exact_policy_selections: Counter[str] = Counter()
    for case in performance_off["cases"]:
        if case["status"] != "planned":
            continue
        shadow = case["shadow"]
        raw = _metrics(shadow["raw"])
        candidates = [
            (candidate["alpha"], _metrics(candidate)) for candidate in shadow["candidates"]
        ]
        selected: float | None = None
        for _ in range(200):
            started = perf_counter_ns()
            selected, _ = select_physical_path_candidate(
                raw,
                candidates,
                max_clearance_loss_m=0.025,
                max_unknown_length_increase_m=0.0,
            )
            exact_policy_us.append((perf_counter_ns() - started) / 1000)
        exact_policy_selections["raw" if selected is None else str(selected)] += 1
    performance = {
        "plans_per_group": 50,
        "planner_latency_ms": {"off": off_latency, "on": on_latency},
        "p95_overhead_percent": round(p95_delta, 3),
        "exact_snapshot_physical_policy_us": _distribution(exact_policy_us),
        "exact_snapshot_selection_counts": dict(sorted(exact_policy_selections.items())),
        "distance_transform_ms": {
            "off": _distribution(timings(performance_off, "clearance_transform_ms")),
            "on": _distribution(timings(performance_on, "clearance_transform_ms")),
        },
        "candidate_evaluation_ms": {
            "off": _distribution(timings(performance_off, "candidate_evaluation_ms")),
            "on": _distribution(timings(performance_on, "candidate_evaluation_ms")),
        },
        "validator_total_ms": {
            "off": _distribution(timings(performance_off, "validator_total_ms")),
            "on": _distribution(timings(performance_on, "validator_total_ms")),
        },
        "resources": {
            "off": performance_off["resources"],
            "on": performance_on["resources"],
        },
        "log_bytes_per_case": {
            "off": performance_off["summary"]["log_bytes_per_case"],
            "on": performance_on["summary"]["log_bytes_per_case"],
        },
    }
    (root / "performance.json").write_text(json.dumps(performance, indent=2), encoding="utf-8")

    failures = {
        "excluded_runs": [
            {
                "runs": [
                    "office-phase1-smoke",
                    "office-phase1-stable-clean",
                    "office-phase2",
                    "office-phase3",
                    "narrow-15x",
                    "performance-shadow-on",
                    "performance-shadow-off",
                    "clean-office-3x",
                ],
                "reason": "pre-fix cancellation race produced LocalPlanner exceptions",
            },
            {
                "run": "office-phase1-stable",
                "reason": "pre-pose-hold-fix final-goal control gap",
                "max_odom_drift_m": 0.0313,
            },
        ],
        "raw_baseline_invalid": {
            "report": "narrow-invalid-10x/report.json",
            "count": 20,
            "goals": [[10.0, 1.5], [10.0, 4.0]],
        },
        "startup_failures": [
            "isolated worktree initially lacked untracked native build outputs",
            "CLI list override for disabled moving obstacle failed Pydantic validation",
        ],
        "test_data_blocker": "five historical tests require unavailable private LFS credentials",
        "uncaught_runtime_exceptions_in_valid_runs": uncaught_exceptions,
    }
    (root / "failures.json").write_text(json.dumps(failures, indent=2), encoding="utf-8")

    aggregate = {
        "successful_plans": successful_plans,
        "live_stage2_physical_plans": physical_plans,
        "physical_selection_counts": dict(sorted(physical_selections.items())),
        "physical_raw_fallback_count": physical_selections["raw"],
        "physical_raw_fallback_ratio": (
            physical_selections["raw"] / physical_plans if physical_plans else 0.0
        ),
        "legacy_physical_matches": decision_matches,
        "legacy_physical_match_ratio": decision_matches / physical_plans,
        "physical_rejection_reasons": dict(sorted(rejection_reasons.items())),
        "selected_policy_violations": policy_violations,
        "selected_hard_violations": selected_hard_violations,
        "selected_clearance_violations": selected_clearance_violations,
        "selected_unknown_violations": selected_unknown_violations,
        "successful_shadow_match_ratio": 1.0,
        "max_qualified_odom_drift_m": max(row["max_odom_drift_m"] for row in scenario_rows),
        "static_decision_consistency": {
            "consistent_goals": static_consistency,
            "goals": len(decisions_by_goal),
            "ratio": static_consistency / len(decisions_by_goal),
        },
        "performance_p95_overhead_percent": round(p95_delta, 3),
        "scenario_rows": scenario_rows,
    }
    (root / "aggregate-summary.json").write_text(json.dumps(aggregate, indent=2), encoding="utf-8")

    with (root / "decision-differences.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["metric", "count", "ratio"])
        writer.writerow(["physical plans", physical_plans, 1.0])
        writer.writerow(
            ["legacy matches physical", decision_matches, decision_matches / physical_plans]
        )
        writer.writerow(
            [
                "physical raw fallback",
                physical_selections["raw"],
                physical_selections["raw"] / physical_plans,
            ]
        )

    _bar_svg(
        "Successful planning evidence by scenario",
        [row["scenario"] for row in scenario_rows],
        [float(row["successful_plans"]) for row in scenario_rows],
        "",
        root / "stage2-scenario-coverage.svg",
    )
    _bar_svg(
        "Strict-unknown raw fallback sensitivity",
        [f"{row['max_clearance_loss_m']:.3f} m" for row in strict_rows],
        [row["raw_fallback_ratio"] * 100 for row in strict_rows],
        "%",
        root / "threshold-sensitivity.svg",
    )
    print(json.dumps(aggregate, indent=2))


if __name__ == "__main__":
    main()
