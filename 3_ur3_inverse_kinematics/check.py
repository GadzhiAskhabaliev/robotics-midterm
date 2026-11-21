"""Numeric check of FK and IK. Does not need MuJoCo."""

from pathlib import Path

import numpy as np

from fk import forward
from ik import solve

Q_GOAL = np.array([0.4, -1.0, 1.1, -1.3, 1.2, 0.5])
Q_START = np.array([0.0, -1.2, 0.6, -0.8, 0.8, 0.0])


def main():
    target = forward(Q_GOAL)
    q, err = solve(Q_START, target)
    lines = [
        "goal joints rad: " + np.array2string(Q_GOAL, precision=6),
        "start joints rad: " + np.array2string(Q_START, precision=6),
        "ik joints rad: " + np.array2string(q, precision=6),
        "target position m: " + np.array2string(target[:3, 3], precision=6),
        "position error m: %.3e" % np.linalg.norm(err[:3]),
        "orientation error rad: %.3e" % np.linalg.norm(err[3:]),
    ]
    text = "\n".join(lines) + "\n"
    out = Path(__file__).resolve().parent / "results" / "check.txt"
    out.parent.mkdir(exist_ok=True)
    out.write_text(text)
    print(text, end="")
    if np.linalg.norm(err[:3]) > 1e-3 or np.linalg.norm(err[3:]) > 1e-2:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
