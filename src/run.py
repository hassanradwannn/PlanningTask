from __future__ import annotations

import argparse
import os
import sys

# Ensure project root is on sys.path when running as a script: e.g., `python src/run.py`
_CURRENT_DIR = os.path.dirname(__file__)
_PROJECT_ROOT = os.path.abspath(os.path.join(_CURRENT_DIR, os.pardir))
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from src.scenarios import get_scenario_names, make_scenario
from src.tester import PathTester


def main() -> None:
    parser = argparse.ArgumentParser(description="Visualize one scenario or all scenarios.")
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument(
        "--scenario",
        type=str,
        default="1",
        choices=get_scenario_names(),
        help="Scenario name to load",
    )
    selection.add_argument("--all", action="store_true", help="Show every scenario in gallery windows")
    parser.add_argument("--sequential", action="store_true",
                        help="With --all, show one scenario at a time; close each plot to continue")
    args = parser.parse_args()

    if args.sequential and not args.all:
        parser.error("--sequential requires --all")
    if args.all:
        PathTester.run_all(sequential=args.sequential)
        return

    cones, car_pose = make_scenario(args.scenario)
    tester = PathTester(cones=cones, car_pose=car_pose)
    tester.run()


if __name__ == "__main__":
    main()

