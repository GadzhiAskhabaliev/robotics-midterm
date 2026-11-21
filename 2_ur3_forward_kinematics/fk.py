"""Modified DH forward kinematics for the UR3 parameters on the task sheet."""

import numpy as np

D1 = 0.152
A2 = -0.244
A3 = -0.213
D4 = 0.131
D5 = 0.085
D6 = 0.092

ALPHA = np.array([0.0, np.pi / 2, 0.0, 0.0, np.pi / 2, -np.pi / 2])
A = np.array([0.0, 0.0, A2, A3, 0.0, 0.0])
D = np.array([D1, 0.0, 0.0, D4, D5, D6])


def link_transform(alpha, a, d, theta):
    ca, sa = np.cos(alpha), np.sin(alpha)
    ct, st = np.cos(theta), np.sin(theta)
    return np.array(
        [
            [ct, -st, 0.0, a],
            [st * ca, ct * ca, -sa, -sa * d],
            [st * sa, ct * sa, ca, ca * d],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )


def forward(q):
    q = np.asarray(q, dtype=float).reshape(6)
    T = np.eye(4)
    for i in range(6):
        T = T @ link_transform(ALPHA[i], A[i], D[i], q[i])
    return T
