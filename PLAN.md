# Path planning submission status

## Current approach

The planner constructs center gates from paired blue/yellow cones and explicitly mirrors unmatched detections. It fits continuous Bézier and interpolating spline candidates, optimizes geometry, then rejects paths that fail clearance, boundary crossing, gate passage, initial heading, or dense/sample curvature checks. See [REPORT.md](REPORT.md).

## Completed

- [x] Preserve the original 20 scenarios and add 12 cases covering three-cone sides and edge conditions.
- [x] Handle zero, one, and multiple visible cones with world-coordinate output.
- [x] Infer missing opposite-side positions from an assumed or observed width.
- [x] Require 0.45 m cone-center clearance and no observed boundary crossings on sampled path segments.
- [x] Add smooth candidate curves through or near center gates, including an inferred approach target.
- [x] Add regression checks for the scenarios reported as problematic.
- [x] Require actual center-gate passage, nonempty straight boundary traversal, and nonoscillating turns in 19/26.
- [x] Show mirrored cones and clearance circles in the viewer.
- [x] Re-run all 32 scenarios numerically and visually.
- [x] Document assumptions and limitations in README and REPORT.

## Submission

Configured repository: https://github.com/hassanradwannn/PlanningTask

Validation complete: 22 automated tests pass and all 32 supplied scenarios pass the independent numerical checks. The report records the remaining vehicle-width and minimum-turning-radius assumptions.
