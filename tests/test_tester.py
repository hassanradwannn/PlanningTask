"""Checks for the gallery and sequential visualization entry points."""
import unittest
from unittest.mock import patch

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from src.run import main
from src.scenarios import get_scenario_names
from src.tester import PathTester


class PathTesterTests(unittest.TestCase):
    def tearDown(self):
        plt.close("all")

    def test_gallery_contains_every_scenario_in_two_figures(self):
        with patch("matplotlib.pyplot.show") as show:
            paths = PathTester.run_all()
        self.assertEqual(list(paths), get_scenario_names())
        self.assertEqual(len(plt.get_fignums()), 2)
        titles = [ax.get_title() for num in plt.get_fignums() for ax in plt.figure(num).axes]
        self.assertEqual(titles, [f"Scenario {name}" for name in get_scenario_names()])
        show.assert_called_once()

    def test_sequential_shows_every_scenario(self):
        with patch("matplotlib.pyplot.show") as show, patch("builtins.print"):
            paths = PathTester.run_all(sequential=True)
        self.assertEqual(list(paths), get_scenario_names())
        self.assertEqual(show.call_count, len(paths))

    def test_cli_all_dispatches_to_gallery(self):
        with patch("sys.argv", ["run", "--all"]), patch.object(PathTester, "run_all") as run_all:
            main()
        run_all.assert_called_once_with(sequential=False)

    def test_cli_all_sequential_dispatches_to_sequential_view(self):
        with patch("sys.argv", ["run", "--all", "--sequential"]), patch.object(PathTester, "run_all") as run_all:
            main()
        run_all.assert_called_once_with(sequential=True)


if __name__ == "__main__":
    unittest.main()
