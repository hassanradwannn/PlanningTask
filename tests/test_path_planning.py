"""Behavioral checks for the planner; run with unittest discovery."""
import math
import random
import unittest

import numpy as np

from scripts.validate_paths import EXPECTED_GATES, boundary_crossings, cone_clearance, gate_errors, turn_metrics
from src.models import CarPose, Cone
from src.path_planning import PathPlanning
from src.scenarios import get_scenario_names, make_scenario


class PathPlanningTests(unittest.TestCase):
    _plans = {}

    def plan(self, cones, car=None):
        car = car or CarPose(0.0, 0.0, 0.0)
        key = (car.x, car.y, car.yaw, tuple((c.x, c.y, c.color) for c in cones))
        if key not in self._plans:
            self._plans[key] = np.array(PathPlanning(car, cones).generatePath())
        return self._plans[key].copy()

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

    def test_reported_problem_cases_clear_cones_and_boundaries(self):
        for name in ("2", "3", "5", "6", "8", "10", "11", "14", "15", "19", "26"):
            with self.subTest(scenario=name):
                cones, car = make_scenario(name)
                path = self.plan(cones, car)
                self.assertGreaterEqual(cone_clearance(path, cones), 0.45 - 1e-8)
                self.assertEqual(boundary_crossings(path, cones), 0)
                turn, curvature = turn_metrics(path)
                self.assertLess(turn, 14.5)
                self.assertLessEqual(curvature, 2.5)
                self.assertTrue(all(error <= 0.15 for error in gate_errors(path, EXPECTED_GATES[name])))

    def test_single_side_routes_choose_the_inside_of_the_boundary(self):
        for name, expected_sign in (("5", 1), ("8", -1)):
            with self.subTest(scenario=name):
                cones, car = make_scenario(name)
                path = self.plan(cones, car)
                boundary_y = cones[0].y
                near_cones = path[(path[:, 0] >= min(c.x for c in cones))
                                  & (path[:, 0] <= max(c.x for c in cones))]
                self.assertGreater(len(near_cones), 5, "Route must actually reach the cone stations")
                self.assertTrue((expected_sign * (near_cones[:, 1] - boundary_y) > 0.45).all())
                self.assertLess(float(np.ptp(near_cones[:, 1])), 0.06)

    def test_missing_blue_is_reflected_to_continue_straight_corridor(self):
        cones, car = make_scenario("6")
        planner = PathPlanning(car, cones)
        path = np.array(planner.generatePath())
        self.assertEqual([(c.x, c.y, c.color) for c in planner.inferred_cones], [(3.0, 4.0, 1)])
        near = path[(path[:, 0] >= 3.0) & (path[:, 0] <= 4.0)]
        self.assertGreater(len(near), 5)
        np.testing.assert_allclose(near[:, 1], 3.0, atol=0.08)

    def test_bends_do_not_oscillate_between_left_and_right_turns(self):
        for name, turn_sign in (("19", -1), ("26", 1)):
            with self.subTest(scenario=name):
                cones, car = make_scenario(name)
                headings = np.unwrap(np.arctan2(*np.diff(self.plan(cones, car), axis=0).T[::-1]))
                turns = turn_sign * np.diff(headings)
                # Ignore less than 0.2 degrees of numerical/endpoint drift.
                self.assertLess(float(np.maximum(-turns - np.deg2rad(0.2), 0).sum()), 0.02)

    def test_infeasible_narrow_gate_is_reported_instead_of_returning_collision(self):
        with self.assertRaisesRegex(ValueError, "No cone-clear"):
            self.plan([Cone(3.0, 0.3, 1), Cone(3.0, -0.3, 0)])

    def test_all_scenarios_have_no_abrupt_heading_jumps(self):
        for name in get_scenario_names():
            with self.subTest(scenario=name):
                cones, car = make_scenario(name)
                path = self.plan(cones, car)
                delta = np.diff(path, axis=0)
                steps = np.linalg.norm(delta, axis=1)
                headings = delta / steps[:, None]
                turns = np.arccos(np.clip(np.sum(headings[:-1] * headings[1:], axis=1), -1, 1))
                self.assertLess(float(np.rad2deg(turns.max())), 14.5)
                # Guard against hiding a tight turn merely by adding samples.
                curvature = turns / ((steps[:-1] + steps[1:]) / 2)
                self.assertLessEqual(float(curvature.max()), 2.5)
                initial = np.array([math.cos(car.yaw), math.sin(car.yaw)])
                self.assertGreater(float(np.dot(headings[0], initial)), math.cos(0.15))


if __name__ == "__main__":
    unittest.main()
