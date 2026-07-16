#!/usr/bin/env python3
"""Run repeatable M20 candidate-validator shadow sweeps over LCM."""

from __future__ import annotations

import argparse
import ast
from collections import Counter
from itertools import pairwise
import json
import math
from pathlib import Path
import time
from typing import Any

import lcm
import numpy as np

from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path as NavPath

CLICK_TOPIC = "/clicked_point#geometry_msgs.PointStamped"
TELEOP_TOPIC = "/tele_cmd_vel#geometry_msgs.Twist"
ODOM_TOPIC = "/dimos/slam_odom#nav_msgs.Odometry"
RAW_TOPIC = "/raw_path#nav_msgs.Path"
PATH_TOPIC = "/path#nav_msgs.Path"
SHADOW_MARKER = "Candidate path validator shadow metrics."

DEFAULT_GOALS = [
    (x, y)
    for y in (-1.5, -0.5, 0.5, 1.5, 2.5, 3.5, 4.5)
    for x in (-5.0, -4.0, -3.0, -2.0, -1.0, 0.0, 1.0)
]


def _parse_shadow_line(line: str) -> dict[str, Any] | None:
    if SHADOW_MARKER not in line:
        return None

    payload = line.split(SHADOW_MARKER, 1)[1].strip()
    if "shadow_report=" in payload:
        encoded = payload.split("shadow_report=", 1)[1].split(" ", 1)[0]
        try:
            if encoded.startswith(("'", '"')):
                encoded = ast.literal_eval(encoded)
            return json.loads(encoded)
        except (json.JSONDecodeError, SyntaxError, ValueError):
            return {"parse_error": line.rstrip()}

    try:
        candidates_text, remainder = payload.removeprefix("candidates=").split(
            " legacy_max_allowed_cost=", 1
        )
        legacy_cost_text, remainder = remainder.split(" raw=", 1)
        raw_text, remainder = remainder.split(" selected_alpha=", 1)
        alpha_text, selected_path = remainder.split(" selected_path=", 1)
        return {
            "candidates": ast.literal_eval(candidates_text),
            "legacy_max_allowed_cost": float(legacy_cost_text),
            "raw": ast.literal_eval(raw_text),
            "selected_alpha": ast.literal_eval(alpha_text),
            "selected_path": selected_path.strip(),
        }
    except (SyntaxError, ValueError):
        try:
            candidates_text, remainder = payload.removeprefix("candidates=").split(" raw=", 1)
            raw_text, remainder = remainder.split(" selected_alpha=", 1)
            alpha_text, selected_path = remainder.split(" selected_path=", 1)
            return {
                "candidates": ast.literal_eval(candidates_text),
                "legacy_max_allowed_cost": None,
                "raw": ast.literal_eval(raw_text),
                "selected_alpha": ast.literal_eval(alpha_text),
                "selected_path": selected_path.strip(),
            }
        except (SyntaxError, ValueError):
            return {"parse_error": line.rstrip()}


def _path_xy(path: NavPath) -> np.ndarray:
    return np.array([[pose.x, pose.y] for pose in path.poses], dtype=np.float64)


def _path_length(path: NavPath) -> float:
    return sum(math.hypot(b.x - a.x, b.y - a.y) for a, b in pairwise(path.poses))


def _path_turn(path: NavPath) -> float:
    headings = [math.atan2(b.y - a.y, b.x - a.x) for a, b in pairwise(path.poses)]
    return sum(abs(math.atan2(math.sin(b - a), math.cos(b - a))) for a, b in pairwise(headings))


def _distribution(values: list[float]) -> dict[str, float] | None:
    if not values:
        return None
    array = np.asarray(values, dtype=np.float64)
    return {
        "min": round(float(np.min(array)), 4),
        "p05": round(float(np.percentile(array, 5)), 4),
        "median": round(float(np.median(array)), 4),
        "p95": round(float(np.percentile(array, 95)), 4),
        "max": round(float(np.max(array)), 4),
    }


def _selected_metrics(shadow: dict[str, Any]) -> dict[str, Any] | None:
    alpha = shadow.get("legacy_selected_alpha", shadow.get("selected_alpha"))
    if alpha is None:
        return None
    return next(
        (
            candidate
            for candidate in shadow.get("candidates", [])
            if math.isclose(float(candidate["alpha"]), float(alpha))
        ),
        None,
    )


def _summarize(cases: list[dict[str, Any]], initial_odom: tuple[float, float]) -> dict[str, Any]:
    successful = [case for case in cases if case["status"] == "planned"]
    shadows = [case["shadow"] for case in successful if "parse_error" not in case["shadow"]]
    selections = Counter(
        (
            "raw"
            if shadow.get("legacy_selected_alpha", shadow.get("selected_alpha")) is None
            else str(shadow.get("legacy_selected_alpha", shadow.get("selected_alpha")))
        )
        for shadow in shadows
    )
    physical_selections = Counter(
        "raw"
        if shadow.get("physical_selected_alpha") is None
        else str(shadow["physical_selected_alpha"])
        for shadow in shadows
        if shadow.get("physical_validator_evaluated")
    )

    deltas: dict[str, list[float]] = {
        "min_clearance_loss_m": [],
        "p5_clearance_loss_m": [],
        "unknown_length_increase_m": [],
        "unknown_ratio_increase": [],
        "path_length_change_m": [],
        "cumulative_turn_change_rad": [],
    }
    hard_invalid_candidates = 0
    unknown_increase_candidates = 0
    physical_policy_violations = 0
    physical_decision_matches = 0
    for shadow in shadows:
        raw = shadow["raw"]
        for candidate in shadow["candidates"]:
            hard_invalid_candidates += candidate["validation_reason"] is not None
            unknown_increase_candidates += (
                candidate["unknown_length_m"] > raw["unknown_length_m"] + 1e-9
            )
            if candidate.get("physical_gate_passed"):
                physical_policy_violations += candidate.get("physical_rejection_reason") is not None

        physical_decision_matches += shadow.get("physical_decision_matches_legacy") is True

        selected = _selected_metrics(shadow)
        if selected is None:
            continue
        if raw["min_clearance_m"] is not None and selected["min_clearance_m"] is not None:
            deltas["min_clearance_loss_m"].append(
                raw["min_clearance_m"] - selected["min_clearance_m"]
            )
        if raw["p5_clearance_m"] is not None and selected["p5_clearance_m"] is not None:
            deltas["p5_clearance_loss_m"].append(raw["p5_clearance_m"] - selected["p5_clearance_m"])
        deltas["unknown_length_increase_m"].append(
            selected["unknown_length_m"] - raw["unknown_length_m"]
        )
        deltas["unknown_ratio_increase"].append(selected["unknown_ratio"] - raw["unknown_ratio"])
        deltas["path_length_change_m"].append(selected["path_length_m"] - raw["path_length_m"])
        deltas["cumulative_turn_change_rad"].append(
            selected["cumulative_turn_rad"] - raw["cumulative_turn_rad"]
        )

    odom_drift = [
        math.hypot(case["odom"][0] - initial_odom[0], case["odom"][1] - initial_odom[1])
        for case in cases
        if case["odom"] is not None
    ]
    return {
        "cases": len(cases),
        "planned": len(successful),
        "status_counts": dict(sorted(Counter(case["status"] for case in cases).items())),
        "no_path_or_timeout": sum(case["status"] == "no_path_or_timeout" for case in cases),
        "shadow_record_mismatch": sum(case["status"] == "shadow_record_mismatch" for case in cases),
        "raw_baseline_invalid": sum(case["status"] == "raw_baseline_invalid" for case in cases),
        "selection_counts": dict(sorted(selections.items())),
        "physical_selection_counts": dict(sorted(physical_selections.items())),
        "physical_decision_matches": physical_decision_matches,
        "physical_policy_violations": physical_policy_violations,
        "max_odom_drift_m": round(max(odom_drift, default=0.0), 4),
        "hard_invalid_candidates": hard_invalid_candidates,
        "unknown_increase_candidates": unknown_increase_candidates,
        "selected_deltas": {name: _distribution(values) for name, values in deltas.items()},
    }


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rounds", type=int, default=3)
    parser.add_argument("--goal-timeout-s", type=float, default=1.5)
    parser.add_argument("--settle-s", type=float, default=0.15)
    parser.add_argument("--initial-wait-s", type=float, default=0.0)
    parser.add_argument("--scenario", default="default-grid")
    parser.add_argument("--phase", default="unspecified")
    parser.add_argument(
        "--goals-file",
        type=Path,
        help="JSON array of [x, y] goals; defaults to the historical 49-goal grid",
    )
    parser.add_argument(
        "--log",
        type=Path,
        default=Path("/tmp/m20_true_simple_nav_sim.log"),
        help="stdout log of the running shadow-enabled simulation",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("/tmp/m20-candidate-validator-sweep"),
    )
    parser.add_argument(
        "--hold-position",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="publish zero teleop before every goal so navigation commands stay suppressed",
    )
    args = parser.parse_args()
    if args.rounds < 1:
        parser.error("--rounds must be at least 1")
    if args.goal_timeout_s <= 0 or args.settle_s < 0 or args.initial_wait_s < 0:
        parser.error("timeouts must be positive and settle time non-negative")
    if args.goals_file is not None and not args.goals_file.is_file():
        parser.error(f"goals file not found: {args.goals_file}")
    return args


def main() -> None:
    args = _arguments()
    if not args.log.is_file():
        raise SystemExit(f"Simulation log not found: {args.log}")

    transport = lcm.LCM()
    goals = DEFAULT_GOALS
    if args.goals_file is not None:
        payload = json.loads(args.goals_file.read_text(encoding="utf-8"))
        goals = [(float(goal[0]), float(goal[1])) for goal in payload]
        if not goals:
            raise SystemExit("Goals file must contain at least one [x, y] goal")
    latest_odom: Odometry | None = None
    latest_raw: NavPath | None = None
    latest_path: NavPath | None = None
    raw_sequence = 0
    path_sequence = 0

    def on_odom(_: str, data: bytes) -> None:
        nonlocal latest_odom
        latest_odom = Odometry.lcm_decode(data)

    def on_raw(_: str, data: bytes) -> None:
        nonlocal latest_raw, raw_sequence
        path = NavPath.lcm_decode(data)
        if path.poses:
            latest_raw = path
            raw_sequence += 1

    def on_path(_: str, data: bytes) -> None:
        nonlocal latest_path, path_sequence
        path = NavPath.lcm_decode(data)
        if path.poses:
            latest_path = path
            path_sequence += 1

    transport.subscribe(ODOM_TOPIC, on_odom)
    transport.subscribe(RAW_TOPIC, on_raw)
    transport.subscribe(PATH_TOPIC, on_path)

    deadline = time.monotonic() + 5.0
    while latest_odom is None and time.monotonic() < deadline:
        transport.handle_timeout(100)
    if latest_odom is None:
        raise SystemExit("No simulation odometry received")
    if args.initial_wait_s:
        time.sleep(args.initial_wait_s)

    args.output.mkdir(parents=True, exist_ok=True)
    initial_odom = (latest_odom.x, latest_odom.y)
    cases: list[dict[str, Any]] = []
    print(f"initial_odom=({initial_odom[0]:.3f}, {initial_odom[1]:.3f})")

    for round_index in range(1, args.rounds + 1):
        for goal_index, (goal_x, goal_y) in enumerate(goals, 1):
            before_raw = raw_sequence
            before_path = path_sequence
            log_offset = args.log.stat().st_size

            if args.hold_position:
                transport.publish(TELEOP_TOPIC, Twist.zero().lcm_encode())
                # Let stop_movement cancellation finish before the next goal.
                time.sleep(0.2)
            transport.publish(
                CLICK_TOPIC,
                PointStamped(goal_x, goal_y, 0.0, frame_id="world").lcm_encode(),
            )

            deadline = time.monotonic() + args.goal_timeout_s
            while time.monotonic() < deadline:
                transport.handle_timeout(50)
                if raw_sequence > before_raw and path_sequence > before_path:
                    break

            log_deadline = time.monotonic() + 0.5
            shadow_records: list[dict[str, Any]] = []
            while time.monotonic() < log_deadline and not shadow_records:
                new_log = args.log.read_bytes()[log_offset:].decode(errors="replace")
                shadow_records = [
                    record
                    for line in new_log.splitlines()
                    if (record := _parse_shadow_line(line)) is not None
                ]
                if not shadow_records:
                    time.sleep(0.02)

            if args.hold_position:
                transport.publish(TELEOP_TOPIC, Twist.zero().lcm_encode())
                # Close the final-goal control gap and let cancellation settle
                # before recording odometry or starting the next case.
                time.sleep(0.2)
                transport.handle_timeout(0)

            odom = latest_odom
            planned = raw_sequence > before_raw and path_sequence > before_path
            if not planned or latest_raw is None or latest_path is None:
                result = {
                    "round": round_index,
                    "case": goal_index,
                    "goal": [goal_x, goal_y],
                    "status": "no_path_or_timeout",
                    "odom": None if odom is None else [round(odom.x, 4), round(odom.y, 4)],
                    "shadow_records": len(shadow_records),
                }
            elif len(shadow_records) != 1:
                result = {
                    "round": round_index,
                    "case": goal_index,
                    "goal": [goal_x, goal_y],
                    "status": "shadow_record_mismatch",
                    "odom": [round(odom.x, 4), round(odom.y, 4)],
                    "shadow_records": len(shadow_records),
                }
            elif shadow_records[0].get("raw_baseline_valid") is False:
                result = {
                    "round": round_index,
                    "case": goal_index,
                    "goal": [goal_x, goal_y],
                    "status": "raw_baseline_invalid",
                    "odom": [round(odom.x, 4), round(odom.y, 4)],
                    "shadow_records": 1,
                    "shadow": shadow_records[0],
                }
            else:
                result = {
                    "round": round_index,
                    "case": goal_index,
                    "goal": [goal_x, goal_y],
                    "status": "planned",
                    "odom": [round(odom.x, 4), round(odom.y, 4)],
                    "raw_length_m": round(_path_length(latest_raw), 4),
                    "output_length_m": round(_path_length(latest_path), 4),
                    "raw_turn_rad": round(_path_turn(latest_raw), 4),
                    "output_turn_rad": round(_path_turn(latest_path), 4),
                    "shadow_records": len(shadow_records),
                    "shadow": shadow_records[-1],
                }
                np.savez_compressed(
                    args.output / f"round-{round_index:02d}-case-{goal_index:02d}.npz",
                    goal=np.array([goal_x, goal_y]),
                    odom=np.array(result["odom"]),
                    raw=_path_xy(latest_raw),
                    output=_path_xy(latest_path),
                )
            cases.append(result)
            print(
                f"round={round_index} case={goal_index:02d} goal=({goal_x:.1f},{goal_y:.1f}) "
                f"status={result['status']} shadow={result['shadow_records']}"
            )
            time.sleep(args.settle_s)

    if args.hold_position:
        transport.publish(TELEOP_TOPIC, Twist.zero().lcm_encode())

    report = {
        "created_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "hold_position": args.hold_position,
        "scenario": args.scenario,
        "phase": args.phase,
        "initial_wait_s": args.initial_wait_s,
        "goals_file": None if args.goals_file is None else str(args.goals_file),
        "rounds": args.rounds,
        "goals_per_round": len(goals),
        "initial_odom": list(initial_odom),
        "summary": _summarize(cases, initial_odom),
        "cases": cases,
    }
    report_path = args.output / "report.json"
    report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report["summary"], indent=2))
    print(f"report={report_path}")


if __name__ == "__main__":
    main()
