from __future__ import annotations

from typing import List

from src.models import CarPose, Cone, Path2D


class PathPlanning:
    """Student-implemented path planner.

    You are given the car pose and an array of detected cones, each cone with (x, y, color)
    where color is 0 for yellow (right side) and 1 for blue (left side). The goal is to
    generate a sequence of path points that the car should follow.

    Implement ONLY the generatePath function.
    """

    def __init__(self, car_pose: CarPose, cones: List[Cone]):
        self.car_pose = car_pose
        self.cones = cones

    def generatePath(self) -> Path2D:
        """Return a list of path points (x, y) in world frame.

        Requirements and notes:
        - Cones: color==0 (yellow) are on the RIGHT of the track; color==1 (blue) are on the LEFT.
        - You may be given 2, 1, or 0 cones on each side.
        - Use the car pose (x, y, yaw) to seed your path direction if needed.
        - Return a drivable path that stays between left (blue) and right (yellow) cones.
        - The returned path will be visualized by PathTester.

        The path can contain as many points as you like, but it should be between 5-10 meters,
        with a step size <= 0.5. Units are meters.

        Use Delaunay gates and a directed graph; infer a 2 m wide track when
        only one boundary is visible. See REPORT.md for assumptions and limits.
        """

        import math
        from itertools import combinations

        import numpy as np
        from scipy.interpolate import BSpline
        from scipy.spatial import Delaunay, QhullError

        horizon, spacing = 8.0, 0.1
        assumed_width, clearance = 2.0, 0.25
        origin = np.array([self.car_pose.x, self.car_pose.y], dtype=float)
        yaw = self.car_pose.yaw
        if not np.isfinite(origin).all() or not math.isfinite(yaw):
            raise ValueError("Car pose must contain finite coordinates and yaw")
        rotation = np.array([[math.cos(yaw), -math.sin(yaw)],
                             [math.sin(yaw), math.cos(yaw)]])

        def unit(vector, default=None):
            length = float(np.linalg.norm(vector))
            if length > 1e-9:
                return vector / length
            return np.array([1.0, 0.0]) if default is None else default.copy()

        def left_normal(tangent):
            return np.array([-tangent[1], tangent[0]])

        def world_path(local):
            return [tuple(map(float, point)) for point in local @ rotation.T + origin]

        def straight():
            return world_path(np.column_stack((np.arange(0.0, horizon + spacing / 2,
                                                         spacing),
                                               np.zeros(int(horizon / spacing) + 1))))

        # Sort before deduplication so triangulation is independent of input order.
        unique = {}
        for cone in sorted(self.cones, key=lambda c: (c.x, c.y, c.color)):
            if cone.color not in (0, 1) or not np.isfinite([cone.x, cone.y]).all():
                continue
            point = (np.array([cone.x, cone.y]) - origin) @ rotation
            key = tuple(np.round(point, 8))
            if key in unique and unique[key][1] != cone.color:
                unique[key] = (point, -1)  # contradictory detection: discard it
            elif key not in unique:
                unique[key] = (point, cone.color)
        detected = [(p, color) for p, color in unique.values() if color >= 0]
        if not detected:
            return straight()
        points = np.array([p for p, _ in detected])
        colors = np.array([color for _, color in detected])
        real_points = points.copy()

        # Fit a common longitudinal axis to within-side variation. Colors choose
        # its sign when both sides exist; yaw chooses it for a single boundary.
        covariance = np.zeros((2, 2))
        for color in (0, 1):
            side = points[colors == color]
            if len(side) > 1:
                centered = side - side.mean(axis=0)
                covariance += centered.T @ centered
        blue, yellow = points[colors == 1], points[colors == 0]
        if np.linalg.norm(covariance) > 1e-9:
            _, eigenvectors = np.linalg.eigh(covariance)
            direction = eigenvectors[:, -1]
        elif len(blue) and len(yellow):
            across = blue.mean(axis=0) - yellow.mean(axis=0)
            direction = unit(np.array([across[1], -across[0]]))
        else:
            direction = np.array([1.0, 0.0])
        if len(blue) and len(yellow):
            if np.dot(left_normal(direction), blue.mean(axis=0) - yellow.mean(axis=0)) < 0:
                direction = -direction
        elif direction[0] < -1e-9 or (abs(direction[0]) <= 1e-9
                                     and np.dot(direction, points.mean(axis=0)) < 0):
            direction = -direction

        # One visible boundary: offset along local normals to create virtual
        # opposite cones. Three cones provide two segments and a bend tangent.
        if not len(blue) or not len(yellow):
            order = np.argsort(points @ direction, kind="stable")
            points, colors = points[order], colors[order]
            inferred = []
            for i, point in enumerate(points):
                tangent = unit(points[min(i + 1, len(points) - 1)]
                               - points[max(i - 1, 0)], direction)
                if np.dot(tangent, direction) < 0:
                    tangent = -tangent
                sign = 1.0 if colors[i] == 0 else -1.0
                inferred.append(point + sign * assumed_width * left_normal(tangent))
            points = np.vstack((points, inferred))
            colors = np.concatenate((colors, 1 - colors))

        gates, gate_ids, links = [], {}, set()

        def add_gate(a, b):
            if colors[a] == colors[b]:
                return None
            edge = tuple(sorted((int(a), int(b))))
            b_idx, y_idx = (a, b) if colors[a] == 1 else (b, a)
            across = points[b_idx] - points[y_idx]
            width = float(np.dot(across, left_normal(direction)))
            # Reject reversed, vanishing and overly longitudinal cross-track edges.
            if width < 2 * clearance or abs(float(np.dot(across, direction))) > 2 * width:
                return None
            if edge not in gate_ids:
                gate_ids[edge] = len(gates)
                gates.append((points[a] + points[b]) / 2)
            return gate_ids[edge]

        triangles = []
        if len(points) >= 3:
            try:
                triangles = Delaunay(points).simplices
            except QhullError:
                # No random jitter: collinear geometry remains collinear.
                pass
        for triangle in triangles:
            ids = [add_gate(a, b) for a, b in combinations(triangle, 2)]
            ids = [idx for idx in ids if idx is not None]
            for a, b in combinations(ids, 2):
                links.add(tuple(sorted((a, b))))
        if not gates:
            # Two cones / degenerate triangulation: ordered compatible gates.
            for a, b in combinations(range(len(points)), 2):
                add_gate(a, b)
            if gates:
                order = sorted(range(len(gates)), key=lambda i: float(np.dot(gates[i], direction)))
                links.update(tuple(sorted((a, b))) for a, b in zip(order, order[1:]))
        if not gates:
            return straight()
        gates = np.array(gates)

        # Each mixed triangle connects the midpoints of its two color-changing
        # edges. Direct those links along the fitted track axis to form a DAG.
        adjacency = [[] for _ in gates]
        for a, b in sorted(links):
            progress = float(np.dot(gates[b] - gates[a], direction))
            if abs(progress) < 1e-8:
                continue
            if progress < 0:
                a, b = b, a
            adjacency[a].append(b)
        starts = [i for i, p in enumerate(gates) if p[0] >= -0.5]
        if not starts:
            return straight()
        # Enter near the car rather than skipping directly to the farthest gate.
        start = min(starts, key=lambda i: (float(np.linalg.norm(gates[i]))
                                          + max(0.0, -float(gates[i][0])), i))
        order = sorted(range(len(gates)), key=lambda i: (float(np.dot(gates[i], direction)), i))
        # State includes the previous gate: turn cost depends on incoming heading.
        states = {(-1, start): (float(np.linalg.norm(gates[start])), [start])}
        for current in order:
            incoming_states = [(key, value) for key, value in states.items() if key[1] == current]
            for (previous, _), (cost, route) in incoming_states:
                incoming = unit(gates[current] if previous < 0 else gates[current] - gates[previous])
                for nxt in adjacency[current]:
                    outgoing = unit(gates[nxt] - gates[current])
                    turn_cost = 2.0 * (1.0 - float(np.clip(np.dot(incoming, outgoing), -1, 1)))
                    candidate = cost + float(np.linalg.norm(gates[nxt] - gates[current])) + turn_cost
                    key = (current, nxt)
                    if key not in states or candidate < states[key][0]:
                        states[key] = (candidate, route + [nxt])
        # Reach the furthest connected gate, then choose the least-cost route.
        _, route = min(states.values(), key=lambda value: (
            -float(np.dot(gates[value[1][-1]] - gates[start], direction)), value[0], value[1]))
        anchors = [np.zeros(2)]
        for idx in route:
            if np.linalg.norm(gates[idx] - anchors[-1]) > 1e-6:
                anchors.append(gates[idx])
        if len(anchors) == 1:
            anchors.append(direction * horizon)
        # Extrapolate linearly, not with an unbounded cubic polynomial.
        final_direction = unit(anchors[-1] - anchors[-2], direction) if len(route) > 1 else direction
        anchors.append(anchors[-1] + horizon * final_direction)
        anchors = np.array(anchors)

        # Checks apply to the visible/inferred boundary segments, not a claimed
        # vehicle footprint. An already-outside car may enter the corridor.
        boundary_segments = []
        for color in (0, 1):
            side = points[colors == color]
            side = side[np.argsort(side @ direction, kind="stable")]
            boundary_segments.extend(zip(side[:-1], side[1:]))

        def acceptable(curve):
            if not np.isfinite(curve).all():
                return False
            a, b = curve[:-1], curve[1:]
            delta = b - a
            squared_lengths = np.maximum(np.sum(delta * delta, axis=1), 1e-12)
            for cone in real_points:
                t = np.clip(np.sum((cone - a) * delta, axis=1) / squared_lengths, 0, 1)
                if np.min(np.linalg.norm(cone - (a + t[:, None] * delta), axis=1)) < clearance - 1e-8:
                    return False
            for c, d in boundary_segments:
                ca, da = c - a, d - a
                ab_c = delta[:, 0] * ca[:, 1] - delta[:, 1] * ca[:, 0]
                ab_d = delta[:, 0] * da[:, 1] - delta[:, 1] * da[:, 0]
                cd_a = (d[0] - c[0]) * (a[:, 1] - c[1]) - (d[1] - c[1]) * (a[:, 0] - c[0])
                cd_b = (d[0] - c[0]) * (b[:, 1] - c[1]) - (d[1] - c[1]) * (b[:, 0] - c[0])
                if np.any((ab_c * ab_d < -1e-10) & (cd_a * cd_b < -1e-10)):
                    return False
            return True

        def resample(curve):
            lengths = np.linalg.norm(np.diff(curve, axis=0), axis=1)
            cumulative = np.concatenate(([0.0], np.cumsum(lengths)))
            keep = np.concatenate(([True], lengths > 1e-10))
            curve, cumulative = curve[keep], cumulative[keep]
            distance = min(horizon, float(cumulative[-1]))
            targets = np.linspace(0.0, distance, int(math.ceil(distance / spacing)) + 1)
            return np.column_stack([np.interp(targets, cumulative, curve[:, axis]) for axis in (0, 1)])

        # Approximate the gate route with a clamped cubic B-spline. Gate
        # midpoints are controls, not mandatory interpolation knots; simple
        # interior knots keep the curve C2 and avoid rapid tangent changes.
        candidates = []
        first_distance = float(np.linalg.norm(anchors[1]))
        entry_direction = unit(anchors[2] - anchors[1], final_direction)

        def consider(handle, approach=0.0, offset=0.0, broad_exit=False, guide_direction=None):
            controls = [anchors[0], np.array([min(handle, first_distance * 0.8), 0.0])]
            if approach:
                guide = entry_direction if guide_direction is None else guide_direction
                controls.append(anchors[1] - approach * guide)
            controls.extend(anchors[1:-1])
            controls.append(anchors[-2] + handle * final_direction
                            + offset * left_normal(final_direction))
            if broad_exit:
                # Extra exit controls give a reversal room to bend instead of
                # collapsing into a near-zero tangent at the last gate.
                controls.append(anchors[-2] + 2 * handle * final_direction
                                + offset * left_normal(final_direction))
                controls.append(anchors[-1] - 0.5 * final_direction)
            controls.append(anchors[-1])
            controls = np.array(controls)
            chord = np.linalg.norm(np.diff(controls, axis=0), axis=1)
            control_parameters = np.concatenate(([0.0], np.cumsum(chord)))
            interior = [np.mean(control_parameters[i:i + 3])
                        for i in range(1, len(controls) - 3)]
            if len(interior) > 1 and np.any(np.diff(interior) <= 1e-9):
                return  # keep interior knots simple for C2 continuity
            knots = np.concatenate((np.zeros(4), interior,
                                    np.full(4, control_parameters[-1])))
            spline = BSpline(knots, controls, 3)
            dense_parameters = np.linspace(0.0, control_parameters[-1],
                                           max(400, int(control_parameters[-1] / 0.015)))
            curve = spline(dense_parameters)
            sampled = resample(curve)
            if not acceptable(sampled):
                return
            velocity = spline(dense_parameters, 1)
            acceleration = spline(dense_parameters, 2)
            speed = np.linalg.norm(velocity, axis=1)
            if speed.min() < 1e-5:
                return  # a vanishing tangent can create a cusp
            curvature = np.abs(velocity[:, 0] * acceleration[:, 1]
                               - velocity[:, 1] * acceleration[:, 0]) / speed**3
            candidates.append((float(curvature.max()), sampled))

        for handle in (0.5, 1.0, 1.5, 2.0):
            consider(handle)
        if not candidates or min(item[0] for item in candidates) > 1.0:
            # A bounded search adjusts entry/exit controls for difficult poses.
            # Every candidate still passes cone and boundary checks.
            reversal = np.dot(unit(anchors[-2] - anchors[-3]), final_direction) < 0
            offsets = (0.0, -0.75, 0.75, -1.5, 1.5) if reversal else (0.0,)
            guides = [entry_direction]
            if np.dot(entry_direction, final_direction) < 0.999:
                guides.append(final_direction)
            for handle in (0.5, 1.5, 3.0):
                for approach in (0.0, 0.75, 1.5):
                    for offset in offsets:
                        for guide in guides if approach else guides[:1]:
                            consider(handle, approach, offset, broad_exit=True,
                                     guide_direction=guide)
        if candidates:
            return world_path(min(candidates, key=lambda item: item[0])[1])
        # No checked smooth connection was found. Keep the heading fallback
        # smooth; the interface has no failure/stop status and this cannot
        # guarantee a clear route through an unknown or inconsistent scene.
        return straight()
