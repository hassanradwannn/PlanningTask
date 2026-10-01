"""Measure all scenarios and save a contact sheet for independent visual review."""
from __future__ import annotations

import argparse
from itertools import permutations
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from src.path_planning import PathPlanning
from src.scenarios import get_scenario_names, make_scenario

# Independent expected corridor stations for the reported regressions. Keeping
# these in validation prevents an omitted or incorrect planner gate from making
# a route that misses the track appear to pass.
EXPECTED_GATES = {
    "2": [(1, 2)], "3": [(1, 2), (3, 2)],
    "5": [(3, 3), (4, 3)], "6": [(3, 3), (4, 3)],
    "8": [(3, 2), (5, 2)], "10": [(5, 2.5)],
    "11": [(1, 2.5), (4, 3.5)], "14": [(3, 3.5), (5, 3.5)],
    "15": [(2, 4), (3, 3)], "19": [(2, 1.5), (5, 2.5)],
    "26": [(2, 0), (4, 0.4), (6, 1.4)],
}


def gate_errors(path, gates):
    a, delta = path[:-1], np.diff(path, axis=0)
    squared = np.maximum(np.sum(delta * delta, axis=1), 1e-12)
    errors = []
    for gate in gates:
        t = np.clip(np.sum((np.array(gate) - a) * delta, axis=1) / squared, 0, 1)
        errors.append(float(np.linalg.norm(np.array(gate) - a - t[:, None] * delta, axis=1).min()))
    return errors


def cone_clearance(path, cones):
    a, b = path[:-1], path[1:]
    delta = b - a
    distances = []
    for cone in cones:
        point = np.array([cone.x, cone.y])
        t = np.clip(np.sum((point - a) * delta, axis=1)
                    / np.maximum(np.sum(delta * delta, axis=1), 1e-12), 0, 1)
        distances.append(float(np.linalg.norm(point - (a + t[:, None] * delta), axis=1).min()))
    return min(distances) if distances else None


def boundary_crossings(path, cones):
    # For these <=3-cone sides, shortest visiting order reconstructs adjacent
    # boundary segments without relying on the planner's fitted direction.
    segments = []
    for color in (0, 1):
        side = list(dict.fromkeys((c.x, c.y) for c in cones if c.color == color))
        if len(side) > 1:
            order = min(permutations(side), key=lambda seq: sum(
                np.linalg.norm(np.array(b) - a) for a, b in zip(seq[:-1], seq[1:])))
            segments.extend((np.array(a), np.array(b)) for a, b in zip(order[:-1], order[1:]))

    def cross(a, b):
        return float(a[0] * b[1] - a[1] * b[0])

    return sum(cross(b - a, c - a) * cross(b - a, d - a) < -1e-10
               and cross(d - c, a - c) * cross(d - c, b - c) < -1e-10
               for a, b in zip(path[:-1], path[1:]) for c, d in segments)


def turn_metrics(path):
    """Heading jumps and curvature estimated from the returned polyline."""
    delta = np.diff(path, axis=0)
    steps = np.linalg.norm(delta, axis=1)
    unit = delta / np.maximum(steps[:, None], 1e-12)
    angles = np.arccos(np.clip(np.sum(unit[:-1] * unit[1:], axis=1), -1, 1))
    curvature = angles / np.maximum((steps[:-1] + steps[1:]) / 2, 1e-12)
    return float(np.rad2deg(angles.max())), float(curvature.max())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("docs/validation"))
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    names = get_scenario_names()
    rows = []
    failures = []
    fig, axes = plt.subplots(int(np.ceil(len(names) / 4)), 4,
                             figsize=(16, 3.5 * int(np.ceil(len(names) / 4))), squeeze=False)
    for name, ax in zip(names, axes.flat):
        cones, car = make_scenario(name)
        planner = PathPlanning(car, cones)
        path = np.array(planner.generatePath())
        steps = np.linalg.norm(np.diff(path, axis=0), axis=1)
        clearance = cone_clearance(path, cones)
        crossings = int(boundary_crossings(path, cones))
        max_turn, max_curvature = turn_metrics(path)
        smooth = max_turn < 14.5 and max_curvature <= 2.5
        errors = gate_errors(path, EXPECTED_GATES.get(name, planner.center_gates))
        gates_passed = all(error <= 0.15 for error in errors)
        valid = bool(np.isfinite(path).all() and np.allclose(path[0], [car.x, car.y])
                     and 5.0 <= steps.sum() <= 10.0 and steps.max() <= 0.5 + 1e-9)
        row = {"scenario": name, "points": len(path), "length_m": float(steps.sum()),
               "max_step_m": float(steps.max()), "min_cone_clearance_m": clearance,
               "observed_boundary_crossings": crossings, "output_contract_passed": valid,
               "max_heading_change_deg": max_turn,
               "estimated_peak_curvature_per_m": max_curvature,
               "gate_errors_m": errors, "gate_checks_passed": gates_passed,
               "inferred_cones": [{"x": c.x, "y": c.y, "color": c.color}
                                  for c in planner.inferred_cones],
               "smoothness_checks_passed": smooth}
        rows.append(row)
        # Geometric checks accompany the output contract on all supplied cases.
        if (not valid or not smooth or not gates_passed
                or (clearance is not None and clearance < 0.45 - 1e-8) or crossings):
            failures.append(name)
        for color, face in ((0, "gold"), (1, "royalblue")):
            side = [c for c in cones if c.color == color]
            ax.scatter([c.x for c in side], [c.y for c in side], c=face, edgecolors="black", s=30)
            inferred = [c for c in planner.inferred_cones if c.color == color]
            ax.scatter([c.x for c in inferred], [c.y for c in inferred],
                       facecolors="none", edgecolors=face, s=30)
        from matplotlib.patches import Circle
        for cone in cones:
            ax.add_patch(Circle((cone.x, cone.y), 0.45, color="gray", alpha=0.12))
        ax.plot(path[:, 0], path[:, 1], color="green")
        ax.scatter([car.x], [car.y], color="red", s=25)
        ax.arrow(car.x, car.y, np.cos(car.yaw), np.sin(car.yaw), color="red", head_width=0.15)
        ax.set_title(f"{name}: {steps.sum():.2f} m; max turn={max_turn:.1f}°")
        ax.set_aspect("equal", adjustable="datalim")
        ax.grid(True, linestyle=":", alpha=0.5)
    for ax in list(axes.flat)[len(names):]:
        ax.set_visible(False)
    fig.tight_layout()
    fig.savefig(args.output / "scenarios.png", dpi=110)
    plt.close(fig)
    (args.output / "metrics.json").write_text(json.dumps(rows, indent=2) + "\n")
    print(f"Validated {len(rows)} scenarios; failures: {failures or 'none'}")
    print(f"Metrics and plots: {args.output}")
    if failures:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
