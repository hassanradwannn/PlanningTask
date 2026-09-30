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

The planner uses **Delaunay triangulation and a directed graph search**. Mixed-color
triangle edges form gates across the track. Their midpoints become graph nodes,
connected through shared triangles. The search follows the estimated track
direction, prefers lower turn costs, and reaches the furthest connected gate.
A cubic Hermite spline connects the car to the selected centerline. The output
starts at the car and contains 33 world-coordinate points covering approximately
8 m at at most 0.25 m spacing on the supplied scenarios.

For one visible side, neighboring cones estimate its tangent and an assumed
**2 m track width** supplies virtual opposite cones. This makes three-cone bends
usable by the same triangulation method. Two-cone or collinear inputs use compatible
gate midpoints directly. No usable forward gates produce a straight heading path.

Smoothing is checked against cone clearance and boundary segments. An unresolved
spline falls back to the graph polyline. Sparse inputs, inferred boundaries, and
inconsistent starting headings do not establish physical vehicle feasibility;
see [the report](REPORT.md) for the exact checks and limitations.

### Validation

Run from the repository root after installing `requirements.txt`:

```bash
python -m unittest discover -s tests -v
python scripts/validate_paths.py
python -m src.run --scenario 23
```

The validation script writes [numerical results](docs/validation/metrics.json)
and [scenario plots](docs/validation/scenarios.png). All 32 supplied cases pass
the output contract, maintain at least 0.25 m clearance from cone centers, and
have no strict crossings of observed boundary segments. The automated suite
contains 14 tests.

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
