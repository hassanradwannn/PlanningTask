## Path Planning Assignment


### Task Description

#### Part 1

You are given up to two cones (sometimes only one, or none), each with coordinates `(x, y)` and a `color` flag:
- `color == 0`: yellow cone (right side of the track)
- `color == 1`: blue cone (left side of the track)

You also receive the car pose `(x, y, yaw)`. Your goal is to return a sequence of path points `(x, y)` in world coordinates, representing a drivable route that stays between the left (blue) and right (yellow) boundaries. The path will be visualized by the tester.

Implement your algorithm in:

```
src/path_planning.py  ->  class PathPlanning.generatePath(self) -> Path2D
```

The environment already runs with a basic straight-line placeholder. Replace it with your solution.

#### Part 2

What if you were given three cones on one side of the track?
- Implement your approach to use the new givens.
- Add new test cases to test your approach in `src/scenarios.py`.
- Explain why you chose this solution and what could be its limitations.

### Quick Start

1. Install Python 3.9+.
2. Create a virtual environment (recommended) and install dependencies:

```
python -m venv .venv
source .venv/bin/activate  # macOS/Linux
# .venv/Scripts/activate   # Windows
pip install -r requirements.txt
```

3. Run a scenario:

```
python -m src.run --scenario 1
```

Available scenarios: numeric names `1`..`32`. Cases `1`..`20` are the unchanged starter inputs; `21`..`27` exercise three-cone boundaries, and `28`..`32` cover additional edge cases. Use `--scenario N` to select.

### Files Overview

- `src/models.py`: data classes for `Cone`, `CarPose`, and `Path2D` alias.
- `src/path_planning.py`: contains `PathPlanning` where you implement `generatePath`.
- `src/tester.py`: simple Matplotlib visualizer that plots cones, the car, heading, and the path.
- `src/scenarios.py`: prebuilt scenarios for testing.
- `src/run.py`: CLI to run a scenario and visualize the result.
- `tests/test_path_planning.py`: automated behavioral checks.
- `scripts/validate_paths.py`: numerical validation and a plot of all scenarios.
- `REPORT.md`: full implementation report, reasoning, assumptions, and results.

### Implemented solution

The planner pairs opposite-color cones into center gates. Unmatched cones use a
local boundary tangent and an inferred opposite side; when no pair supplies a
width, it assumes a **2 m track**. This also handles three cones along a bend.

It compares quintic Bézier routes with degree-four/five spline routes through
the gates. Unmatched cones are explicitly reflected using a nearby observed
blue–yellow pair, or a 2 m assumed width when no pair exists. Inferred cones
appear as hollow markers in the viewer; gray discs show the 0.35 m margin.

With both boundary colors available, the curve may pass anywhere inside the
gate opening after insetting each end by the clearance margin. This gives it
room to take a gentler turn without requiring an exact midpoint. The curve
must actually cross every safe gate opening. Entirely missing sides retain
their assumed centerline.

The cost penalizes observed boundary crossings, close passes to cones, missed
gates, tight curvature, and backtracking. Final selection rejects candidates
that violate clearance, gate passage, initial heading, or curvature checks.
Curves join the straight continuation with zero curvature. All candidates begin
along the car's yaw. Routes are approximately 8 m long, or 10 m when the last
target lies behind the current heading, with at most 0.1 m point spacing.
The same planner handles every scenario without case-specific paths. See
[the report](REPORT.md) for the cost, assumptions, and limits.

### Validation

Run from the repository root after installing `requirements.txt`:

```bash
python -m unittest discover -s tests -v
python scripts/validate_paths.py
python -m src.run --scenario 23
python -m src.run --all
python -m src.run --all --sequential
```

`--all` opens two gallery windows with 16 scenarios each. For larger individual
plots, add `--sequential` and close each plot to continue to the next scenario.

The validation script writes [numerical results](docs/validation/metrics.json)
and [scenario plots](docs/validation/scenarios.png). It checks the output
contract, at least 0.35 m clearance from cone centers, zero strict crossings
of observed boundary segments, actual crossing of the safe gate openings,
and bounded heading change and curvature.
The automated suite includes regressions for the reported problem scenes.
See the [before/after comparison for 2, 3, and 14](docs/validation/clearance_comparison.png).

The current solution uses direct cone pairing and curve optimization rather
than the earlier Delaunay graph. It produces continuous curves, but a real
car's minimum turning radius and width still need to be supplied before
claiming the route is physically drivable. If no curve passes the checks,
the planner raises `ValueError` instead of returning a colliding candidate.

If ROS has added unrelated packages to `PYTHONPATH`, run commands with
`env -u PYTHONPATH`. For a headless machine, use `MPLBACKEND=Agg`; if Matplotlib's
default cache is not writable, set `MPLCONFIGDIR=/tmp/path-planning-mpl`.

### What to Submit

- Upload your solution to github and submit your repository link.
- Don't forget to add documentation, explaining your solution.

### Author's Note
- Keep it simple.
- This task was chosen to give you a sense of how things work, you don't have to implement complex algorithms, read research papers, or spend days thinking about it, just implement the simplest approach you can think of that can return a correct path.
- You are free to assume any missing information, but mention it in your submission.
- Try to solve as many cases as you can, but you don't have to solve all the problems that can ever exist, just try your best.
- Reminder: This problem is based on FSAI competition (a simplified version), it does not represent how path planning works everywhere it just provides an example to give you a better understanding of what path planning can look like.
- Enjoy :)
