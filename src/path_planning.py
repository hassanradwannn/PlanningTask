from __future__ import annotations

from typing import List

from src.models import CarPose, Cone, Path2D


class PathPlanning:
    """Plan a short centerline from colored cone boundaries."""

    def __init__(self, car_pose: CarPose, cones: List[Cone]):
        self.car_pose = car_pose
        self.cones = cones
        self.inferred_cones: List[Cone] = []
        self.center_gates: Path2D = []

    def generatePath(self) -> Path2D:
        """Return an 8–10 m path in world coordinates, sampled every 0.1 m.

        Blue marks the left edge and yellow the right. A missing edge is
        inferred using observed cross-track offsets or a 2 m track width.
        Raise ValueError if no candidate passes the geometric acceptance checks.
        """
        import math

        import numpy as np
        from scipy.interpolate import make_interp_spline
        from scipy.optimize import linear_sum_assignment, minimize

        spacing, horizon, assumed_width = 0.1, 8.0, 2.0
        self.inferred_cones = []
        self.center_gates = []
        origin = np.array((self.car_pose.x, self.car_pose.y), dtype=float)
        yaw = self.car_pose.yaw
        if not np.isfinite(origin).all() or not math.isfinite(yaw):
            raise ValueError("Car pose must contain finite coordinates and yaw")
        heading = np.array((math.cos(yaw), math.sin(yaw)))

        def unit(vector, fallback):
            length = float(np.linalg.norm(vector))
            return vector / length if length > 1e-9 else fallback.copy()

        def left(vector):
            return np.array((-vector[1], vector[0]))

        def straight():
            distances = np.arange(81) * spacing
            return [tuple(map(float, point)) for point in origin + distances[:, None] * heading]

        # Stable cleanup makes duplicates and input ordering immaterial.
        unique = {}
        for cone in sorted(self.cones, key=lambda c: (c.x, c.y, c.color)):
            if cone.color not in (0, 1) or not np.isfinite((cone.x, cone.y)).all():
                continue
            key = (round(float(cone.x), 8), round(float(cone.y), 8))
            if key in unique and unique[key] != cone.color:
                unique[key] = -1
            elif key not in unique:
                unique[key] = cone.color
        blue = np.array([p for p, color in unique.items() if color == 1], dtype=float).reshape(-1, 2)
        yellow = np.array([p for p, color in unique.items() if color == 0], dtype=float).reshape(-1, 2)
        if not len(blue) and not len(yellow):
            return straight()

        # Estimate the travel axis from variation along each boundary.
        covariance = np.zeros((2, 2))
        for side in (blue, yellow):
            if len(side) > 1:
                centered = side - side.mean(axis=0)
                covariance += centered.T @ centered
        if np.linalg.norm(covariance) > 1e-9:
            _, vectors = np.linalg.eigh(covariance)
            direction = vectors[:, -1]
        elif len(blue) and len(yellow):
            across = blue.mean(axis=0) - yellow.mean(axis=0)
            direction = unit(np.array((across[1], -across[0])), heading)
        else:
            direction = heading.copy()
        if len(blue) and len(yellow):
            across = blue.mean(axis=0) - yellow.mean(axis=0)
            lateral = float(np.dot(left(direction), across))
            if abs(lateral) < 0.25:
                return straight()
            if lateral < 0:
                direction = -direction
        else:
            side = blue if len(blue) else yellow
            if np.dot(side.mean(axis=0) - origin, direction) < 0:
                direction = -direction

        # Pair opposing observations at similar longitudinal stations. Create
        # center targets from unmatched cones by offsetting their own boundary.
        gates = []
        paired_blue, paired_yellow, widths, across_pairs = set(), set(), [], []
        if len(blue) and len(yellow):
            bp = (blue - origin) @ direction
            yp = (yellow - origin) @ direction
            rows, cols = linear_sum_assignment(np.abs(bp[:, None] - yp[None, :]))
            for bi, yi in zip(rows, cols):
                across = blue[bi] - yellow[yi]
                width = float(np.dot(across, left(direction)))
                along = abs(float(np.dot(across, direction)))
                if width < 0.5 or along > max(1.5, width):
                    continue
                gates.append((blue[bi] + yellow[yi]) / 2)
                widths.append(width)
                across_pairs.append(((blue[bi] + yellow[yi]) / 2, across))
                paired_blue.add(int(bi))
                paired_yellow.add(int(yi))
        width = float(np.clip(np.median(widths) if widths else assumed_width, 0.8, 4.0))

        def infer(side, color, paired):
            if not len(side):
                return
            order = np.argsort((side - origin) @ direction, kind="stable")
            ordered = side[order]
            for place, index in enumerate(order):
                if int(index) in paired:
                    continue
                before = ordered[max(0, place - 1)]
                after = ordered[min(len(ordered) - 1, place + 1)]
                tangent = unit(after - before, direction)
                if np.dot(tangent, direction) < 0:
                    tangent = -tangent
                if across_pairs:
                    # Reflect an unmatched observation using the nearest real
                    # cross-track pair. This keeps an uneven detection count
                    # from inventing a diagonal gate (e.g. a missing blue cone).
                    _, across = min(across_pairs, key=lambda pair:
                                    abs(float(np.dot(side[index] - pair[0], direction))))
                else:
                    across = width * left(tangent)
                mirrored = side[index] + (-1 if color == 1 else 1) * across
                self.inferred_cones.append(Cone(float(mirrored[0]), float(mirrored[1]), 1 - color))
                gates.append((side[index] + mirrored) / 2)

        infer(blue, 1, paired_blue)
        infer(yellow, 0, paired_yellow)
        if not gates:
            return straight()
        gates = np.array(gates)
        gates = gates[np.argsort((gates - origin) @ direction, kind="stable")]
        if max(np.dot(gates[-1] - origin, direction),
               np.dot(gates[-1] - origin, heading)) < 0.2:
            return straight()
        self.center_gates = [tuple(map(float, gate)) for gate in gates]
        last_gate = gates[-1]
        final_direction = direction
        if len(gates) > 1:
            candidate = unit(gates[-1] - gates[-2], direction)
            if np.dot(candidate, direction) >= 0.25:
                final_direction = candidate
        # A target behind the current heading needs room for a forward turn.
        # The assignment permits up to 10 m of path in that geometry.
        if np.dot(last_gate - origin, heading) < 0:
            horizon = 10.0
        target = last_gate + 2.0 * final_direction

        # Compete several smooth curve families under the same geometric cost.
        # The spline can pass exactly through gates; an approach knot gives it
        # room to enter a narrow first gate from outside the visible boundary.
        cones = np.vstack((blue, yellow))
        dense_t = np.linspace(0.0, 1.0, 321)
        basis = np.array([math.comb(5, i) * (1 - dense_t) ** (5 - i) * dense_t ** i
                          for i in range(6)]).T
        margin = 0.46
        curvature_limit = 1.25
        boundary_segments = []
        for side in (blue, yellow):
            if len(side) > 1:
                ordered = side[np.argsort((side - origin) @ direction, kind="stable")]
                boundary_segments.extend(zip(ordered[:-1], ordered[1:]))

        def bezier(handles):
            # Collinear endpoint controls give zero endpoint curvature, so
            # appending the straight continuation does not change steering
            # suddenly. This is a quintic, not a chain of short cubic pieces.
            controls = np.array((origin, origin + handles[0] * heading,
                                 origin + 2 * handles[0] * heading,
                                 target - 2 * handles[1] * final_direction,
                                 target - handles[1] * final_direction, target))
            return basis @ controls

        def sampled_path(curve):
            if not np.isfinite(curve).all():
                return None
            lengths = np.linalg.norm(np.diff(curve, axis=0), axis=1)
            if lengths.min() < 1e-9:
                return None
            cumulative = np.concatenate(([0.0], np.cumsum(lengths)))
            if cumulative[-1] < horizon:
                extension = horizon - cumulative[-1] + spacing
                tail = target + np.arange(1, int(math.ceil(extension / spacing)) + 1)[:, None] * spacing * final_direction
                curve = np.vstack((curve, tail))
                cumulative = np.concatenate((cumulative, cumulative[-1]
                                             + spacing * np.arange(1, len(tail) + 1)))
            samples = np.arange(int(round(horizon / spacing)) + 1) * spacing
            path = np.column_stack((np.interp(samples, cumulative, curve[:, 0]),
                                    np.interp(samples, cumulative, curve[:, 1])))
            return path

        def curve_cost(curve, require_feasible=False):
            path = sampled_path(curve)
            if path is None:
                return math.inf if require_feasible else 1e8
            score = 0.0
            feasible = True
            a, b = path[:-1], path[1:]
            delta = b - a
            delta_sq = np.maximum(np.sum(delta * delta, axis=1), 1e-12)
            for gate in gates:
                distance = float(np.linalg.norm(path - gate, axis=1).min())
                feasible &= distance <= 0.15
                score += 1e6 * max(0.0, distance - 0.08) ** 2
            for cone in cones:
                projection = np.clip(np.sum((cone - a) * delta, axis=1) / delta_sq, 0, 1)
                distance = float(np.linalg.norm(cone - (a + projection[:, None] * delta), axis=1).min())
                feasible &= distance >= 0.45
                score += 1e6 * max(0.0, margin - distance) ** 2
            # A boundary crossing is a route failure, not a minor smoothing
            # preference. Check the actual segments the tester will draw.
            for c, d in boundary_segments:
                ab_c = delta[:, 0] * (c[1] - a[:, 1]) - delta[:, 1] * (c[0] - a[:, 0])
                ab_d = delta[:, 0] * (d[1] - a[:, 1]) - delta[:, 1] * (d[0] - a[:, 0])
                cd_a = (d[0] - c[0]) * (a[:, 1] - c[1]) - (d[1] - c[1]) * (a[:, 0] - c[0])
                cd_b = (d[0] - c[0]) * (b[:, 1] - c[1]) - (d[1] - c[1]) * (b[:, 0] - c[0])
                crossings = np.count_nonzero((ab_c * ab_d < -1e-10)
                                            & (cd_a * cd_b < -1e-10))
                feasible &= crossings == 0
                score += 1e6 * crossings
            speed = np.linalg.norm(delta, axis=1)
            directions = delta / np.maximum(speed[:, None], 1e-12)
            turns = np.arccos(np.clip(np.sum(directions[:-1] * directions[1:], axis=1), -1, 1))
            curvature = turns / np.maximum((speed[:-1] + speed[1:]) / 2, 1e-12)
            score += float(np.mean(curvature ** 2))
            score += 10000.0 * max(0.0, float(curvature.max()) - curvature_limit) ** 2
            score += 25.0 * max(0.0, float(np.linalg.norm(np.diff(curve, axis=0), axis=1).sum())
                                 - horizon + 0.1) ** 2
            backtracking = np.maximum(0.0, -(delta @ direction)).sum()
            score += 5.0 * backtracking ** 2
            turning = delta[:-1, 0] * delta[1:, 1] - delta[:-1, 1] * delta[1:, 0]
            signed_turns = np.sign(turning) * turns
            reversal = min(np.maximum(signed_turns - 0.002, 0).sum(),
                           np.maximum(-signed_turns - 0.002, 0).sum())
            score += 100.0 * reversal ** 2
            initial_error = math.acos(float(np.clip(directions[0] @ heading, -1, 1)))
            feasible &= initial_error <= 0.15 and float(curvature.max()) <= 2.5
            score += 1e6 * max(0.0, initial_error - 0.05) ** 2
            # Dense geometry catches a cusp that uniform 0.1 m output samples
            # could skip. Sampling more finely must not hide a tight turn.
            dense_delta = np.diff(curve, axis=0)
            dense_speed = np.linalg.norm(dense_delta, axis=1)
            dense_unit = dense_delta / np.maximum(dense_speed[:, None], 1e-12)
            dense_angles = np.arccos(np.clip(np.sum(dense_unit[:-1] * dense_unit[1:], axis=1), -1, 1))
            dense_curvature = dense_angles / np.maximum((dense_speed[:-1] + dense_speed[1:]) / 2, 1e-12)
            feasible &= float(dense_curvature.max()) <= 2.5
            score += 10000.0 * max(0.0, float(dense_curvature.max()) - curvature_limit) ** 2
            if require_feasible and not feasible:
                return math.inf
            return float(score)

        distance = float(np.linalg.norm(target - origin))
        limit = max(1.0, min(6.0, distance + 2.0))
        bounds = ((0.12, limit), (0.12, limit))
        candidates = []

        def fit(builder, seeds, search_bounds=bounds):
            seeds = [tuple(np.clip(seed, np.array(search_bounds)[:, 0],
                                   np.array(search_bounds)[:, 1])) for seed in seeds]
            ranked = sorted(((curve_cost(builder(seed)), seed) for seed in seeds),
                            key=lambda item: item[0])
            for _, seed in ranked[:2]:
                result = minimize(lambda values: curve_cost(builder(values)), seed,
                                  method="Powell", bounds=search_bounds,
                                  options={"maxiter": 65, "xtol": 2e-3, "ftol": 1e-3})
                candidates.extend((builder(seed), builder(result.x)))

        fit(bezier, [(a, b) for a in (0.5, 1.5, 3.0, 5.0)
                     for b in (0.5, 1.5, 3.0, 5.0)])

        def spline_builder(nodes, degree=5):
            chord = np.linalg.norm(np.diff(nodes, axis=0), axis=1)
            knots = np.concatenate(([0.0], np.cumsum(chord)))
            def build(handles):
                parameters = knots / knots[-1]
                start = [(1, degree * handles[0] * heading)]
                if degree == 5:
                    start.append((2, np.zeros(2)))
                spline = make_interp_spline(parameters, nodes, k=degree, axis=0,
                                            bc_type=(start,
                                                     [(1, degree * handles[1] * final_direction),
                                                      (2, np.zeros(2))]))
                return spline(dense_t)
            return build

        seeds = [(a, b) for a in (0.3, 0.75, 1.5, 3.0)
                 for b in (0.3, 0.75, 1.5, 3.0)]
        if len(gates):
            nodes = np.vstack((origin, gates, target))
            if np.min(np.linalg.norm(np.diff(nodes, axis=0), axis=1)) > 1e-6:
                for degree in (4, 5):
                    fit(spline_builder(nodes, degree), seeds)
        progress = float(np.dot(gates[0] - origin, direction))
        if progress > 0.2:
            for approach in (0.5, 1.0, 1.5, 2.0):
                entry = gates[0] - approach * direction
                nodes = np.vstack((origin, entry, gates, target))
                if np.min(np.linalg.norm(np.diff(nodes, axis=0), axis=1)) > 1e-6:
                    for degree in (4, 5):
                        fit(spline_builder(nodes, degree), seeds)

        first_direction = unit(gates[0] - origin, heading)
        if np.dot(first_direction, heading) < math.cos(1.0):
            def free_bezier(values):
                controls = np.array((origin, origin + values[0] * heading,
                                     origin + values[2] * direction + values[3] * left(direction),
                                     target - 2 * values[1] * final_direction,
                                     target - values[1] * final_direction, target))
                return basis @ controls
            free_seeds = [(a, b, float(2 * a * heading @ direction),
                           float(2 * a * heading @ left(direction)))
                          for a in (0.5, 1.5, 3.0) for b in (0.5, 1.5, 3.0)]
            fit(free_bezier, free_seeds, (*bounds, (-limit, limit), (-limit, limit)))
            for departure in (0.5, 1.0, 1.5):
                for approach in (0.0, 1.0):
                    nodes = [origin, origin + departure * heading]
                    if approach:
                        nodes.append(gates[0] - approach * direction)
                    nodes = np.vstack((*nodes, gates, target))
                    if np.min(np.linalg.norm(np.diff(nodes, axis=0), axis=1)) > 1e-6:
                        fit(spline_builder(nodes), seeds)

        curve = min(candidates, key=lambda candidate: curve_cost(candidate, require_feasible=True))
        if not math.isfinite(curve_cost(curve, require_feasible=True)):
            raise ValueError(f"No cone-clear, smooth candidate fits the {horizon:g} m planning horizon")
        path = sampled_path(curve)
        return [tuple(map(float, point)) for point in path]
