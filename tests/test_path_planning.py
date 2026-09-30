"""Behavioral checks for the planner; run with unittest discovery."""
import math
import random
import unittest
from unittest.mock import patch

import numpy as np
from scipy.spatial import Delaunay, QhullError

from src.models import CarPose, Cone
from src.path_planning import PathPlanning
from src.scenarios import get_scenario_names, make_scenario


class PathPlanningTests(unittest.TestCase):
    def plan(self, cones, car=None):
        return np.array(PathPlanning(car or CarPose(0.0, 0.0, 0.0), cones).generatePath())

    def assert_contract(self, path, car):
        self.assertEqual(path.shape[1], 2)
        self.assertTrue(np.isfinite(path).all())
        np.testing.assert_allclose(path[0], [car.x, car.y], atol=1e-9)
        steps = np.linalg.norm(np.diff(path, axis=0), axis=1)
        self.assertTrue((steps > 0).all())
        self.assertLessEqual(float(steps.max()), 0.5 + 1e-9)
        self.assertGreaterEqual(float(steps.sum()), 5.0)
        self.assertLessEqual(float(steps.sum()), 10.0)

    def test_all_scenarios_satisfy_output_contract(self):
        for name in get_scenario_names():
            with self.subTest(scenario=name):
                cones, car = make_scenario(name)
                self.assert_contract(self.plan(cones, car), car)

    def test_no_cones_follow_exact_world_heading(self):
        car = CarPose(3.0, -2.0, 0.7)
        distances = np.linspace(0.0, 8.0, 81)
        expected = np.array([car.x, car.y]) + distances[:, None] * [math.cos(car.yaw), math.sin(car.yaw)]
        np.testing.assert_allclose(self.plan([], car), expected, atol=1e-10)

    def test_single_pair_uses_gate_midpoint(self):
        path = self.plan([Cone(4.0, 1.0, 1), Cone(4.0, -1.0, 0)])
        np.testing.assert_allclose(path[:, 1], 0.0, atol=1e-9)
        self.assertLess(float(np.linalg.norm(path - [4.0, 0.0], axis=1).min()), 0.26)

    def test_single_cone_offsets_inward_for_both_colors(self):
        for color, y in ((1, 1.0), (0, -1.0)):
            with self.subTest(color=color):
                path = self.plan([Cone(3.0, y, color)])
                np.testing.assert_allclose(path[:, 1], 0.0, atol=1e-9)

    def test_three_cone_straight_boundaries_have_exact_centerline(self):
        for name in ("21", "22", "27", "32"):
            with self.subTest(scenario=name):
                cones, car = make_scenario(name)
                path = self.plan(cones, car)
                np.testing.assert_allclose(path[:, 1], 0.0, atol=1e-9)
                self.assertAlmostEqual(path[-1, 0], 8.0, places=8)

    def test_three_cone_bends_are_used_and_mirror_each_other(self):
        blue, car = make_scenario("23")
        yellow, _ = make_scenario("24")
        left = self.plan(blue, car)
        right = self.plan(yellow, car)
        self.assertGreater(left[-1, 1], 1.0)
        self.assertTrue((np.diff(left[:, 0]) > 0).all())
        np.testing.assert_allclose(left * [1.0, -1.0], right, atol=1e-8)

    def test_rotation_and_translation_preserve_curved_route(self):
        cones, car = make_scenario("23")
        angle, shift = 0.7, np.array([3.0, -2.0])
        rotation = np.array([[math.cos(angle), -math.sin(angle)],
                             [math.sin(angle), math.cos(angle)]])
        transformed = [Cone(*(np.array([c.x, c.y]) @ rotation.T + shift), c.color) for c in cones]
        expected = self.plan(cones, car) @ rotation.T + shift
        actual = self.plan(transformed, CarPose(*shift, angle))
        np.testing.assert_allclose(actual, expected, atol=1e-8)

    def test_input_order_does_not_change_route(self):
        cones, car = make_scenario("26")
        expected = self.plan(cones, car)
        rng = random.Random(42)
        for _ in range(5):
            rng.shuffle(cones)
            np.testing.assert_allclose(self.plan(cones, car), expected, atol=1e-10)

    def test_duplicates_do_not_change_route(self):
        cones = [Cone(2.0, 1.0, 1), Cone(4.0, 1.0, 1),
                 Cone(2.0, -1.0, 0), Cone(4.0, -1.0, 0)]
        np.testing.assert_allclose(self.plan(cones + [cones[0]]), self.plan(cones), atol=1e-10)

    def test_behind_cones_use_forward_fallback(self):
        cones, car = make_scenario("29")
        np.testing.assert_allclose(self.plan(cones, car), self.plan([], car), atol=1e-10)

    def test_invalid_detections_and_conflicting_colors_are_discarded(self):
        bad = [Cone(float("nan"), 0.0, 1), Cone(1.0, float("inf"), 0),
               Cone(1.0, 1.0, 2), Cone(2.0, 2.0, 0), Cone(2.0, 2.0, 1)]
        np.testing.assert_allclose(self.plan(bad), self.plan([]), atol=1e-10)

    def test_nonfinite_car_pose_is_rejected(self):
        for car in (CarPose(float("nan"), 0.0, 0.0), CarPose(0.0, 0.0, float("inf"))):
            with self.assertRaises(ValueError):
                self.plan([], car)

    def test_three_cone_case_actually_calls_delaunay(self):
        cones, car = make_scenario("23")
        with patch("scipy.spatial.Delaunay", wraps=Delaunay) as triangulate:
            self.plan(cones, car)
        self.assertEqual(triangulate.call_count, 1)
        self.assertEqual(triangulate.call_args.args[0].shape, (6, 2))

    def test_qhull_failure_has_deterministic_fallback(self):
        cones, car = make_scenario("21")
        with patch("scipy.spatial.Delaunay", side_effect=QhullError("degenerate geometry")):
            path = self.plan(cones, car)
        self.assert_contract(path, car)
        np.testing.assert_allclose(path[:, 1], 0.0, atol=1e-9)

    def test_all_scenarios_have_no_abrupt_heading_jumps(self):
        for name in get_scenario_names():
            with self.subTest(scenario=name):
                cones, car = make_scenario(name)
                path = self.plan(cones, car)
                delta = np.diff(path, axis=0)
                steps = np.linalg.norm(delta, axis=1)
                headings = delta / steps[:, None]
                turns = np.arccos(np.clip(np.sum(headings[:-1] * headings[1:], axis=1), -1, 1))
                self.assertLess(float(np.rad2deg(turns.max())), 15.0)
                # Guard against hiding a tight turn merely by adding samples.
                curvature = turns / ((steps[:-1] + steps[1:]) / 2)
                self.assertLess(float(curvature.max()), 3.0)
                initial = np.array([math.cos(car.yaw), math.sin(car.yaw)])
                self.assertGreater(float(np.dot(headings[0], initial)), math.cos(0.15))


if __name__ == "__main__":
    unittest.main()
