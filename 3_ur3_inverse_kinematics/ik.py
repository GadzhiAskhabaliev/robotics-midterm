"""Damped least squares inverse kinematics on the forward model in fk.py."""

import numpy as np

from fk import forward


def rotation_vector(R):
    c = float(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))
    angle = np.arccos(c)
    if angle < 1e-12:
        return np.zeros(3)
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return v * (angle / (2.0 * np.sin(angle)))


def residual(q, target):
    T = forward(q)
    position = target[:3, 3] - T[:3, 3]
    orientation = rotation_vector(target[:3, :3] @ T[:3, :3].T)
    return np.concatenate([position, orientation])


def jacobian(q, target, eps=1e-6):
    base = residual(q, target)
    J = np.zeros((6, 6))
    for i in range(6):
        step = np.zeros(6)
        step[i] = eps
        J[:, i] = (base - residual(q + step, target)) / eps
    return J


def solve(q0, target, damping=0.05, max_step=0.2, iterations=60):
    q = np.asarray(q0, dtype=float).reshape(6).copy()
    target = np.asarray(target, dtype=float)
    for _ in range(iterations):
        e = residual(q, target)
        if np.linalg.norm(e[:3]) < 1e-4 and np.linalg.norm(e[3:]) < 1e-3:
            break
        J = jacobian(q, target)
        A = J @ J.T + (damping ** 2) * np.eye(6)
        dq = J.T @ np.linalg.solve(A, e)
        n = np.linalg.norm(dq)
        if n > max_step:
            dq *= max_step / n
        q = q + dq
    return q, residual(q, target)
