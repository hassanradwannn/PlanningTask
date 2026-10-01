from __future__ import annotations

from typing import Dict, List

import matplotlib.pyplot as plt

from src.models import CarPose, Cone, Path2D
from src.path_planning import PathPlanning


class PathTester:
    """Utility to visualize cones, car pose, and the planned path."""

    def __init__(self, cones: List[Cone], car_pose: CarPose):
        self.cones = cones
        self.car_pose = car_pose

    def run(self) -> Path2D:
        planner = PathPlanning(self.car_pose, self.cones)
        path = planner.generatePath()

        self._plot_scene(path, planner=planner)
        return path

    @classmethod
    def run_all(cls, sequential: bool = False) -> Dict[str, Path2D]:
        """Show all scenarios in two galleries, or one window at a time."""
        import math
        from matplotlib.lines import Line2D
        from src.scenarios import get_scenario_names, make_scenario

        names = get_scenario_names()
        paths = {}
        if sequential:
            for name in names:
                cones, car = make_scenario(name)
                tester = cls(cones, car)
                planner = PathPlanning(car, cones)
                path = planner.generatePath()
                paths[name] = path
                print(f"Scenario {name}/{len(names)} — close the plot to continue")
                tester._plot_scene(path, title=f"Scenario {name}", planner=planner)
                plt.close()
            return paths

        for start in range(0, len(names), 16):
            page = names[start:start + 16]
            fig, axes = plt.subplots(math.ceil(len(page) / 4), 4,
                                     figsize=(16, 13), squeeze=False)
            fig.suptitle(f"Path planning — scenarios {page[0]}–{page[-1]}", fontsize=16)
            for name, ax in zip(page, axes.flat):
                cones, car = make_scenario(name)
                planner = PathPlanning(car, cones)
                path = planner.generatePath()
                paths[name] = path
                cls(cones, car)._plot_scene(path, ax=ax, title=f"Scenario {name}",
                                            show=False, legend=False, planner=planner)
            for ax in list(axes.flat)[len(page):]:
                ax.set_visible(False)
            fig.legend(handles=[
                Line2D([], [], marker="o", linestyle="", color="royalblue", label="Blue (left)"),
                Line2D([], [], marker="o", linestyle="", color="gold", label="Yellow (right)"),
                Line2D([], [], marker="o", linestyle="", color="red", label="Car / heading"),
                Line2D([], [], color="limegreen", label="Planned path"),
            ], loc="upper center", bbox_to_anchor=(0.5, 0.965), ncol=4)
            fig.tight_layout(rect=(0, 0, 1, 0.93))
        plt.show()
        return paths

    def _plot_scene(self, path: Path2D, *, ax=None, title=None,
                    show: bool = True, legend: bool = True, planner=None) -> None:
        from matplotlib.patches import Circle
        if ax is None:
            _, ax = plt.subplots(figsize=(8, 6))
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True, linestyle=":", linewidth=0.5)
        ax.set_xlabel("X [m]")
        ax.set_ylabel("Y [m]")
        ax.set_title(title or "FSAI-Style Cone Track Path Planning Test")
        # Include the complete route and translated/rotated scenarios.
        inferred = planner.inferred_cones if planner else []
        visible = ([(self.car_pose.x, self.car_pose.y)]
                   + [(c.x, c.y) for c in self.cones + inferred] + path)
        ax.set_xlim(min(p[0] for p in visible) - 1.0, max(p[0] for p in visible) + 1.0)
        ax.set_ylim(min(p[1] for p in visible) - 1.0, max(p[1] for p in visible) + 1.0)

        # Plot cones by color
        yellow_x = [c.x for c in self.cones if c.color == 0]
        yellow_y = [c.y for c in self.cones if c.color == 0]
        blue_x = [c.x for c in self.cones if c.color == 1]
        blue_y = [c.y for c in self.cones if c.color == 1]

        if yellow_x:
            ax.scatter(yellow_x, yellow_y, c="gold", edgecolors="black", label="Yellow (Right)")
        if blue_x:
            ax.scatter(blue_x, blue_y, c="royalblue", edgecolors="black", label="Blue (Left)")
        for color, edge in ((0, "goldenrod"), (1, "royalblue")):
            side = [c for c in inferred if c.color == color]
            if side:
                ax.scatter([c.x for c in side], [c.y for c in side], facecolors="none",
                           edgecolors=edge, marker="o", label="Inferred opposite side")
        for cone in self.cones:
            ax.add_patch(Circle((cone.x, cone.y), 0.45, color="gray", alpha=0.12))

        # Plot car pose and heading arrow
        ax.scatter([self.car_pose.x], [self.car_pose.y], c="red", s=60, marker="o", label="Car")
        self._draw_heading_arrow(ax)

        # Plot path if available
        if path:
            px = [p[0] for p in path]
            py = [p[1] for p in path]
            ax.plot(px, py, "-", color="limegreen", linewidth=2.0, label="Planned Path")

        if legend:
            ax.legend(loc="best")

        if show:
            plt.show()

    def _draw_heading_arrow(self, ax) -> None:
        import math

        length = 1.0
        dx = math.cos(self.car_pose.yaw) * length
        dy = math.sin(self.car_pose.yaw) * length
        ax.arrow(self.car_pose.x, self.car_pose.y, dx, dy, head_width=0.3, head_length=0.4, fc="red", ec="red")
