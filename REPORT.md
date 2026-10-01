# Cone path planning report

## Task and deliverables

The planner returns world-coordinate `(x, y)` points starting at the car pose. Blue cones mark the left boundary, yellow cones the right. Paths cover approximately 8 m, or 10 m when the car initially faces away from the corridor, with point spacing at most 0.1 m.

The submission includes:

1. `src/path_planning.py`: the planner for zero, one, two, and three cones per side.
2. `src/scenarios.py`: the original 20 cases, unchanged, plus 12 cases for three-cone sides, curves, missing observations, translated poses, and degenerate inputs.
3. `src/tester.py` and `src/run.py`: individual, gallery, and sequential visualization.
4. `tests/`: behavioral and viewer tests.
5. `scripts/validate_paths.py`: independent geometry checks and saved plots/results.
6. This report and `README.md`: how to run, what changed, why, and limitations.

## What changed after the reported failures

| Scenarios | Correction |
|---|---|
| 2, 3, 10, 11, 15 | Check distance from every returned path segment to cone centers, reject collisions and boundary crossings, and require actual crossing of safe gate openings. |
| 5 | Infer blue cones above the yellow boundary; enter the corridor smoothly and continue straight at `y = 3`. |
| 6 | Reflect the unmatched yellow cone `(3, 2)` using the observed cross-track vector `(0, 2)`. Its missing blue partner is `(3, 4)`, producing the center gate `(3, 3)`. |
| 8 | Infer yellow cones below the blue boundary; continue at `y = 2`. |
| 14 | Allow a 10 m approach from the original yaw and require crossing the gates at `x = 3` and `x = 5` between the yellow and blue boundaries. Clearance without reaching the track no longer counts as success. |
| 19 | Penalize unnecessary changes between left and right turning and test that the right-turning route does not oscillate. |
| 26 | Reflect the missing yellow cone to `(6, 0.4)`, use the resulting gate `(6, 1.4)`, and test a continuous left bend. |

The viewer now draws inferred cones as hollow markers. Gray discs show the 0.35 m cone-center clearance used for acceptance. These discs are a geometric margin, not a measured car footprint.

## Smaller cone margin and gentler turns

The requested adjustment reduced the hard clearance from 0.45 m to **0.35 m**. Reducing the margin alone was insufficient: the previous route was also forced almost through every exact gate midpoint. With both colors observed, a gate is now treated as an opening, inset by the clearance margin at both ends. The route must intersect that safe opening, but may choose a lateral position that reduces curvature. Every path segment still has to pass the cone-clearance and boundary-crossing checks.

The midpoint remains a soft preference. For an entirely missing side, the assumed centerline is retained because its width is unobserved. For collinear targets, an extra approach candidate can finish at the first target with zero curvature and continue straight along the inferred centerline. Straightness within that section is a preference rather than a hard condition, which preserves gentler approaches to diagonal tracks. This makes scenario 8's observed section exactly straight.

The curvature preference was strengthened to target at most 1.0 per metre. A flexible terminal offset and an intermediate Bézier control point allow broader turns; the fixed-exit search is retained as an additional candidate so a new optimization variable cannot remove its good solutions.

| Scenario | Previous maximum heading change | Updated maximum | Reduction | Updated minimum cone-center clearance |
|---|---:|---:|---:|---:|
| 2 | 10.34° | 6.48° | 37% | 0.354 m |
| 3 | 7.09° | 5.33° | 25% | 0.360 m |
| 14 | 12.51° | 5.72° | 54% | 0.422 m |

These heading changes are between adjacent returned segments at approximately 0.1 m spacing. Curvature also decreases in all three cases, so the improvement does not come from adding denser output points. All three still cross their safe cone gates and have zero observed-boundary crossings. The comparison uses the previous planner from commit `2132f9d`; scenario inputs are unchanged.

[Before/after plots](docs/validation/clearance_comparison.png) and [comparison measurements](docs/validation/clearance_comparison.json) record the result.

## How the planner works

### 1. Clean observations and estimate track direction

Invalid coordinates/colors are discarded. Duplicate positions are merged; conflicting colors at one position are discarded. Variation within the observed boundaries gives the longitudinal track axis. With both colors, the axis is oriented so blue is on the left. For a single side, it points toward the observed cones. Initial path direction still comes from the car's yaw.

This separates track direction from car heading: a car looking toward a boundary does not reverse the meaning of blue and yellow.

### 2. Pair cones and mirror missing partners

Minimum-cost assignment pairs opposite colors at similar longitudinal positions. Each accepted pair defines a midpoint gate and an observed cross-track vector.

For an unmatched cone, the nearest observed pair supplies the reflection vector. If no pair exists, the local boundary tangent supplies an inward normal and the assumed track width is 2 m. Neighboring tangents allow three cones on one side to describe a bend.

In scenario 6:

```text
observed blue (4, 4) - observed yellow (4, 2) = (0, 2)
missing blue = unmatched yellow (3, 2) + (0, 2) = (3, 4)
center = ((3, 2) + (3, 4)) / 2 = (3, 3)
```

### 3. Fit continuous curves through the corridor

The search compares quintic Bézier curves and degree-four/five interpolating splines. Endpoint handles control initial and terminal tangents. Geometry-derived approach/departure knots give a turn more room when the car is poorly aligned with the first gate. A free intermediate Bézier control point provides another candidate for those difficult approaches.

The endpoint lies 2 m beyond the last gate, with an optional lateral offset within its safe width. This lets the curve pass through that gate without having to finish turning exactly there. The curve ends with zero curvature; any straight continuation therefore joins without a sudden steering change. Spline pieces also have continuous derivatives at their internal knots. There are no manually drawn scenario-specific paths.

The dense curve is sampled at 321 parameter values and resampled at approximately equal 0.1 m arc-length intervals for output.

### 4. Optimize, then reject invalid candidates

Powell optimization varies endpoint handles and, for the free-control family, an intermediate control point and optional lateral exit offset. Costs penalize missed gate openings, close cone passes, observed-boundary crossings, excessive length, backward progress, curvature, initial-heading error, and unnecessary turn reversals. Distance from a gate's center remains a small preference when both colors are available.

A weighted cost alone is insufficient: a candidate can trade a collision for a slightly smoother turn. Final selection therefore accepts only candidates satisfying:

- at least **0.35 m** from every observed cone center to every returned path segment;
- **zero strict crossings** of observed boundary segments;
- an actual returned-segment crossing of every gate opening after insetting each end by **0.35 m**;
- for an entirely inferred side, a returned sample within **0.15 m** of each assumed center target;
- first-segment heading error at most **0.15 rad** from the car yaw;
- peak estimated curvature at most **2.5 per metre**, checked on both the dense curve and returned path.

Optimization targets 0.36 m clearance to leave room for final acceptance. Curvature above 1.0 per metre is penalized: this corresponds to a preferred 1 m turning radius, not a guaranteed radius. Gates narrower than twice the clearance margin are rejected. If no candidate passes acceptance, the planner raises `ValueError` instead of returning the least-bad colliding route.

With no usable cones, cones wholly behind the car, or an ambiguous collinear corridor, the original heading-based straight fallback remains. These fallback geometries are separate from a failed curve search.

## Why this approach

The earlier implementation used Delaunay gate connectivity followed by smoothing. The current workspace had already moved to direct pairing and curve optimization; this correction builds on that implementation. **The current version does not use Delaunay triangulation or graph search.** For these sparse local scenes, pairing gives explicit center targets, while continuous-curve search directly addresses gate passage and steering. Delaunay graph search remains a possible extension when substantially more cones or branches are available.

The previous checks were too permissive. A route could miss the corridor, and side checks could pass with an empty collection of samples near the cones. The revised checks require actual gate passage and nonempty samples along the straight single-side boundaries. Dense-curve checks also prevent a tiny cusp between output samples from being hidden by resampling.

## Validation results

All 32 supplied scenarios pass the output, clearance, boundary-crossing, gate-passage, and sampled smoothness checks. All 24 automated tests pass. The suite also verifies the specific reported routes, reflected missing cone, absence of turn oscillations in 19/26, input-order independence, duplicate handling, coordinate transformations, infeasible-gate reporting, and both viewer modes. Added checks enforce the improved turns in 2/3/14 and detect a collision between waypoints, even when the endpoints are clear.

Measured across the supplied cases:

| Measurement | Result |
|---|---:|
| Minimum observed-cone center clearance | 0.354 m |
| Strict observed-boundary crossings | 0 |
| Maximum adjacent heading change at 0.1 m spacing | 7.63 degrees |
| Maximum estimated returned-path curvature | 1.333 per metre |
| Tightest estimated turning radius | about 0.750 m |
| Scenario 19 maximum adjacent heading change | 5.22 degrees |
| Scenario 26 maximum adjacent heading change | 1.00 degrees |

Scenario 16 now has the largest estimated curvature; scenario 14 is approximately 1.0 per metre after the adjustment. No scene worsened by more than 0.25 degrees in maximum adjacent heading change versus the previous run. The curves have continuous tangents and curvature, but the preferred 1 m minimum radius is not met in every case. No vehicle turning-radius specification was supplied; these results must not be described as a physical steering guarantee.

Saved evidence:

- [All scenario plots](docs/validation/scenarios.png)
- [Per-scenario measurements, gate crossing results, midpoint offsets, and inferred cones](docs/validation/metrics.json)

## Run and inspect

From the submission directory:

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

## Assumptions and limitations

- Cone positions and clearances refer to centers. Car width, cone radius, wheelbase, speed, and maximum steering angle are unspecified. A real vehicle requires corresponding clearance and curvature limits.
- A missing entire side uses an assumed 2 m track width. Incorrect width shifts its inferred centerline. Mirrored cones are inferred, not sensor observations.
- PCA direction and longitudinal pairing assume a short local section. They do not solve arbitrary loops, hairpins, branches, or incorrect cone colors.
- Only visible cones and segments joining same-color neighbors are checked. Unseen obstacles and track continuation remain unknown.
- Dense sampling estimates curvature; it is not an analytic proof of a bound everywhere on the continuous curve.
- The finite candidate search can fail even when some other valid route exists. Its error means no accepted candidate was found, not that the geometry is mathematically impossible.
- Multi-start numerical optimization is appropriate for this assignment's small scenes; no real-time execution deadline is guaranteed.
