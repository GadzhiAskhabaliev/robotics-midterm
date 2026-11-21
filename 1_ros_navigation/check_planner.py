"""Checks GridPlanner without ROS or the simulator."""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent / "RoboticsLesson_ws/src/navigation_pkg"))

from navigation_pkg.planner import GridPlanner


def crosses_wall(path, wall_x=1.0, y_min=-2.0, y_max=2.0):
    for a, b in zip(path, path[1:]):
        if (a[0] - wall_x) * (b[0] - wall_x) > 0:
            continue
        if a[0] == b[0]:
            y = a[1]
        else:
            t = (wall_x - a[0]) / (b[0] - a[0])
            y = a[1] + t * (b[1] - a[1])
        if y_min <= y <= y_max:
            return True
    return False


def main():
    planner = GridPlanner()

    straight = planner.planning((0.0, 0.0), (2.0, 0.0), None)
    assert np.allclose(straight[0], (0.0, 0.0))
    assert np.allclose(straight[-1], (2.0, 0.0))
    assert len(straight) > 2

    wall = np.array([[1.0, y] for y in np.linspace(-2.0, 2.0, 41)])
    around = planner.planning((0.0, 0.0), (2.0, 0.0), wall)
    assert np.allclose(around[0], (0.0, 0.0))
    assert np.allclose(around[-1], (2.0, 0.0))
    assert not crosses_wall(around), around

    goals = [np.array([-3.0, 3.0]), np.array([7.0, -10.0]), np.array([6.0, -6.0])]
    pose = np.array([0.0, 0.0])
    for goal in goals:
        path = planner.planning(pose, goal, wall)
        assert np.allclose(path[0], pose)
        assert np.allclose(path[-1], goal)
        assert not crosses_wall(path)
        pose = goal

    print("planner check passed")
    print("empty-map path points:", len(straight))
    print("around-wall path points:", len(around))


if __name__ == "__main__":
    main()
