# Cone path planning report

## Algorithm used

**Cone-gate path planning using Bézier/spline curves and Powell optimization.**

This is an **optimization-based local path planner**. It constructs a track corridor from the observed cones, generates smooth candidate curves, optimizes their geometry, and selects a curve that passes explicit clearance and corridor checks.

| Component | Method | Purpose |
|---|---|---|
| Track direction | Principal component analysis (PCA) | Estimate the direction along the observed boundaries. |
| Opposing-cone pairing | Minimum-cost assignment | Match blue and yellow cones at similar longitudinal positions. |
| Missing boundary | Geometric reflection and local boundary normals | Infer opposite-side cones from measured separation or assumed width. |
| Candidate paths | Quintic Bézier curves and degree-four/five interpolating splines | Represent continuous turns with controlled initial and terminal tangents. |
| Curve search | Bounded Powell optimization from multiple starting values | Adjust control points and endpoint handles to reduce turning and corridor costs. |
| Final acceptance | Explicit geometric checks | Reject candidates that violate clearance, gate passage, heading, or curvature requirements. |

The implementation uses direct cone pairing and numerical curve optimization. Delaunay triangulation and graph search are not components of this pipeline.

## Task and deliverables

The planner returns world-coordinate `(x, y)` points starting at the car pose. Blue cones mark the left boundary; yellow cones mark the right. Paths cover approximately 8 m, or 10 m when the last gate lies behind the car's initial heading, with point spacing at most approximately 0.1 m.

The submission includes:

1. `src/path_planning.py`: the planner for zero, one, two, and three cones per side.
2. `src/scenarios.py`: the original 20 cases, unchanged, plus 12 cases for three-cone sides, curves, missing observations, translated poses, and degenerate inputs.
3. `src/tester.py` and `src/run.py`: individual, gallery, and sequential visualization.
4. `tests/`: behavioral and viewer tests.
5. `scripts/validate_paths.py`: independent geometry checks and saved plots/results.
6. This report and `README.md`: the algorithm, design rationale, assumptions, limitations, and run instructions.

## Planning pipeline

### 1. Clean observations and estimate track direction

Invalid coordinates and colors are discarded. Duplicate positions are merged; conflicting colors at one position are discarded.

When a boundary has multiple cones, PCA finds the dominant variation along the boundaries. The covariance is calculated within each color so separation across the track does not dominate the direction estimate. With one cone on each side, the blue–yellow separation supplies a perpendicular track direction. With a single cone, the car heading supplies the direction estimate.

With both colors, the travel direction is oriented so blue lies on the left and yellow on the right. For a single side, its sign points toward the visible cones. The car yaw determines the initial path tangent independently of the inferred track direction.

**Reason:** the observed road direction and the direction the car currently faces can differ. Keeping them separate preserves the meaning of the cone colors while allowing a smooth approach from the actual pose.

### 2. Pair opposing cones and infer missing partners

Minimum-cost assignment pairs blue and yellow cones according to their separation along the estimated track axis. Implausible pairs are rejected using their lateral width and longitudinal separation. Each accepted pair defines a gate, its midpoint, and an observed cross-track vector.

For an unmatched cone, the nearest observed pair supplies a reflection vector. If no pair exists, the local boundary tangent supplies an inward normal and the assumed track width is 2 m. Neighboring tangents allow three cones on one side to describe a bend.

**Reason:** matching by longitudinal station identifies opposing observations. Reflecting a missing partner from measured separation preserves the local track width and avoids introducing a diagonal gate solely because one side has fewer detections.

For example, scenario 6 illustrates the reflection calculation:

```text
observed blue (4, 4) - observed yellow (4, 2) = (0, 2)
inferred blue = unmatched yellow (3, 2) + (0, 2) = (3, 4)
center target = ((3, 2) + (3, 4)) / 2 = (3, 3)
```

These coordinates illustrate the general calculation. The planner derives them from its inputs; it does not contain a scenario-6 branch.

### 3. Define safe gate openings

Each gate is the segment connecting opposing cones. Its endpoints are inset by **0.35 m** to define a safe opening. A valid route must actually cross every safe opening.

When both colors are observed, the midpoint is a soft preference: the curve may choose another position within the safe opening to reduce curvature. With an entirely missing side, proximity to the assumed center targets is required because the true track width is unobserved.

**Reason:** a gate represents available track space. Allowing lateral movement within that space gives the car room for gentler turns while preserving explicit cone clearance and passage between the boundaries.

### 4. Generate smooth candidate curves

The search compares quintic Bézier curves and degree-four/five interpolating splines. Endpoint handles control the initial and terminal tangents. Approach and departure knots are calculated from the track direction, car heading, and gate positions. Free-control candidates also vary an intermediate Bézier point and, when both sides are observed, a lateral exit offset.

The endpoint lies 2 m beyond the last gate. Curves end with zero curvature so a straight continuation joins smoothly. Spline pieces have continuous derivatives at their internal knots.

For collinear targets on a single observed side, an approach candidate can end at the first center target and continue straight along the inferred centerline. Straightness in that section is a preference, allowing other candidates to retain a gentler approach when necessary.

**Reason:** several curve families provide different ways to approach the corridor. The extension beyond the last gate gives the curve space to finish turning. Retaining both fixed and flexible exit candidates reduces dependence on one optimization search finding a good local solution.

### 5. Optimize and select an accepted candidate

Powell optimization varies the curve parameters within bounds. Several starting values are evaluated, and the best starting candidates in each family are optimized.

The cost penalizes:

- missed safe gate openings and distance from nominal center targets;
- insufficient cone clearance and observed-boundary crossings;
- tight curvature and unnecessary reversals between left and right turning;
- excessive route length and backward progress;
- initial-heading error;
- departures from a straight inferred centerline when its targets are collinear.

Final selection applies explicit acceptance checks in addition to the cost:

| Requirement | Acceptance criterion |
|---|---|
| Cone clearance | At least **0.35 m** from every observed cone center to every returned path segment. |
| Observed boundaries | **Zero strict crossings** of segments connecting same-color neighbors. |
| Gate passage | An actual returned-segment crossing of every safe gate opening. |
| Entirely inferred side | A returned sample within **0.15 m** of each assumed center target. |
| Initial heading | First-segment heading error at most **0.15 rad** from the car yaw. |
| Curvature | Peak estimated curvature at most **2.5 per metre**, checked on the dense curve and returned path. |

Gates narrower than twice the clearance margin are rejected. If no candidate passes acceptance, the planner raises `ValueError`.

**Reason:** weighted costs allow tradeoffs between objectives. Separate acceptance checks ensure that a low-cost candidate cannot be selected by sacrificing a clearance or corridor requirement.

### 6. Sample the path for output

Each continuous candidate is evaluated at 321 parameter values and resampled at approximately equal 0.1 m arc-length intervals. Scoring and clearance checks use the returned path segments that the tester displays. Dense-curve curvature checks detect tight features that output sampling could miss.

With no usable cones, no forward gate along either the inferred track direction or car heading, or an ambiguous collinear corridor, the planner returns a straight path along the car heading. This is an explicit fallback for insufficient usable corridor information.

## Design decisions and rationale

| Decision | Value or policy | Why it is used |
|---|---|---|
| Local planning method | Direct pairing followed by curve optimization | The task supplies few cones in a short local section. Pairing creates explicit gates, and curve optimization addresses turning geometry directly. |
| Minimum cone-center clearance | **0.35 m** | Provides positive separation while leaving lateral room within narrow gates. A 1 m gate retains a 0.30 m safe opening. This is a model assumption, not a measured vehicle footprint. |
| Optimization clearance target | **0.36 m** | Gives the search a small buffer above the acceptance threshold. |
| Preferred curvature | At most **1.0 per metre** | Penalizes turns tighter than a preferred 1 m radius. It is a preference; the separate acceptance cap is 2.5 per metre. |
| Missing-side width | **2 m** when no measured pair is available | Supplies an explicit width assumption for otherwise underdetermined single-side scenes. |
| Planning length | **8 m**, or **10 m** for a gate behind the initial heading | Provides useful continuation and more room for an approach from an unfavorable orientation, within the assignment's 5–10 m range. |
| Output spacing | Approximately **0.1 m** | Provides finer geometric representation than the assignment's 0.5 m maximum spacing. |
| Terminal curvature | Zero | Connects the fitted curve to straight continuation without a curvature jump. |
| Gate position | Flexible within the safe opening when both colors are observed | Allows curvature reduction while keeping the route between the cones. |
| Inferred-cone display | Hollow markers | Makes estimated boundary positions distinguishable from sensor observations. |

Gray discs in the viewer show the same 0.35 m clearance used by the planner and validation script.

## General algorithm and scenario coverage

`PathPlanning` receives only the car pose and cone observations. It does not receive a scenario number or read predefined paths. Curve parameters and approach targets are calculated from the input geometry.

Scenario-specific expected gates in the validation script are independent reference data used to check outputs. They are not supplied to the planner. Fixed widths, margins, curve families, and cost weights are shared planning parameters.

Representative cases illustrate the decisions:

| Scenarios | Planning behavior illustrated |
|---|---|
| 2, 3, 10, 11, 15 | Safe gate passage and clearance checks along complete path segments. |
| 5 and 8 | Inferred opposite boundaries and straight continuation on the correct side of the observed cones. |
| 6 | Reflection of an unmatched yellow cone to infer its blue partner. |
| 14 | A 10 m approach from a heading initially facing away from the corridor. |
| 19 | A turning cost that discourages unnecessary left/right oscillation. |
| 26 | A missing yellow partner inferred along a curved track. |
| 21–27 | Use of three observations on a side to describe straight and curved boundaries. |
| 28–32 | Translated poses, cones behind the car, ambiguous geometry, and duplicate observations. |

## Validation results

All 32 supplied scenarios pass the output, clearance, boundary-crossing, gate-passage, and sampled smoothness checks. All 24 automated tests pass.

The tests cover the representative routes, reflected missing cones, turn behavior in 19/26, input-order independence, duplicate handling, coordinate transformations, infeasible-gate reporting, and viewer modes. Clearance is checked between waypoints as well as at the points themselves. Straight-section checks require nonempty samples within the observed cone stations.

Measured across the supplied cases:

| Measurement | Result |
|---|---:|
| Minimum observed-cone center clearance | 0.354 m |
| Strict observed-boundary crossings | 0 |
| Maximum adjacent heading change at approximately 0.1 m spacing | 7.63 degrees |
| Maximum estimated returned-path curvature | 1.333 per metre |
| Tightest estimated turning radius | About 0.750 m |
| Scenario 19 maximum adjacent heading change | 5.22 degrees |
| Scenario 26 maximum adjacent heading change | 1.00 degrees |

Selected gate-approach results:

| Scenario | Maximum adjacent heading change | Minimum cone-center clearance |
|---|---:|---:|
| 2 | 6.48° | 0.354 m |
| 3 | 5.33° | 0.360 m |
| 14 | 5.72° | 0.422 m |

Scenario 16 has the largest estimated curvature; scenario 14 is approximately 1.0 per metre. The preferred 1 m minimum radius is not met in every case. These measurements describe the supplied scenes and do not establish a physical steering guarantee.

Saved evidence:

- [All scenario plots](docs/validation/scenarios.png)
- [Per-scenario measurements, gate crossing results, midpoint offsets, and inferred cones](docs/validation/metrics.json)

## Run and inspect

From the submission directory:

### macOS / Linux

```bash
# All cases in two gallery windows
env -u PYTHONPATH .venv/bin/python -m src.run --all

# Large plots one at a time; close each window to advance
env -u PYTHONPATH .venv/bin/python -m src.run --all --sequential

# Inspect one case
env -u PYTHONPATH .venv/bin/python -m src.run --scenario 6

# Automated checks and saved validation plots
env -u PYTHONPATH .venv/bin/python -m unittest discover -s tests -v
env -u PYTHONPATH .venv/bin/python scripts/validate_paths.py
```

If dependencies are missing, install `requirements.txt` into the configured virtual environment. On a headless machine use `MPLBACKEND=Agg`; set `MPLCONFIGDIR=/tmp/path-planning-mpl` if the default Matplotlib cache is not writable. Headless mode saves validation plots but cannot open interactive windows.

### Windows (PowerShell)

Run from the repository root with dependencies already installed in `.venv`:

```powershell
# Automated tests
.\.venv\Scripts\python.exe -m unittest discover -s tests -v

# Numerical checks and saved validation plots
.\.venv\Scripts\python.exe scripts/validate_paths.py
```

## Assumptions and limitations

- Cone positions and clearances refer to centers. Car width, cone radius, wheelbase, speed, and maximum steering angle are unspecified. A real vehicle requires corresponding clearance and curvature limits.
- A missing entire side uses an assumed 2 m track width. Incorrect width shifts its inferred centerline. Mirrored cones are estimated boundary positions.
- PCA direction and longitudinal pairing assume a short local section. They do not solve arbitrary loops, hairpins, branches, or incorrect cone colors.
- Only visible cones and segments joining same-color neighbors are checked. Unseen obstacles and track continuation remain unknown.
- Dense sampling estimates curvature; it is not an analytic proof of a bound everywhere on the continuous curve.
- The finite candidate search can fail even when another valid route exists. An error means no accepted candidate was found, not that the geometry is mathematically impossible.
- The straight fallback cannot establish clearance when usable corridor information is absent or ambiguous.
- Multi-start numerical optimization is appropriate for this assignment's small scenes; no real-time execution deadline is guaranteed.
