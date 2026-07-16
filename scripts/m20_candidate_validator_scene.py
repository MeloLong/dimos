#!/usr/bin/env python3
"""Generate deterministic MuJoCo occupancy scenes for validator testing."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

RESOLUTION_M = 0.05
WIDTH_CELLS = 240
HEIGHT_CELLS = 200
CORRIDORS = (
    ("blocked_0.95m", 1.5, 0.95),
    ("critical_1.00m", 4.0, 1.00),
    ("generous_1.20m", 6.5, 1.20),
)


def _cell(value_m: float) -> int:
    return round(value_m / RESOLUTION_M)


def build_narrow_passage_scene() -> np.ndarray:
    grid = np.full((HEIGHT_CELLS, WIDTH_CELLS), 100, dtype=np.int8)

    # Shared staging area lets one fixed robot pose address every corridor.
    grid[_cell(0.4) : _cell(9.6), _cell(0.4) : _cell(3.0)] = 0
    for _, center_y_m, width_m in CORRIDORS:
        half_cells = _cell(width_m) // 2
        center_cell = _cell(center_y_m)
        grid[
            center_cell - half_cells : center_cell - half_cells + _cell(width_m),
            _cell(2.5) : _cell(11.5),
        ] = 0

    # A wide arena supports obstacle-adjacent straight, turn, S, and V goals.
    grid[_cell(7.2) : _cell(9.6), _cell(2.5) : _cell(11.5)] = 0
    grid[_cell(7.2) : _cell(8.25), _cell(5.0) : _cell(5.5)] = 100
    grid[_cell(8.55) : _cell(9.6), _cell(7.0) : _cell(7.5)] = 100
    return grid


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--metadata", type=Path)
    args = parser.parse_args()

    args.output.parent.mkdir(parents=True, exist_ok=True)
    np.save(args.output, build_narrow_passage_scene())
    metadata = {
        "resolution_m": RESOLUTION_M,
        "shape": [HEIGHT_CELLS, WIDTH_CELLS],
        "robot_start": [1.5, 4.0],
        "planning_robot_width_m": 0.9,
        "planning_inflation_factor": 1.1,
        "effective_planning_width_m": 0.99,
        "corridors": [
            {"name": name, "center_y_m": center, "width_m": width}
            for name, center, width in CORRIDORS
        ],
    }
    if args.metadata is not None:
        args.metadata.parent.mkdir(parents=True, exist_ok=True)
        args.metadata.write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    print(json.dumps(metadata))


if __name__ == "__main__":
    main()
