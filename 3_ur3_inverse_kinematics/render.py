"""Put the FK and IK joint vectors on the official UR3 model and save two views."""

from pathlib import Path

import numpy as np

from check import Q_GOAL, Q_START
from fk import forward
from ik import solve

ROOT = Path(__file__).resolve().parent
JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


def quat_from_matrix(R):
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2.0
        q = np.array(
            [
                0.25 * s,
                (R[2, 1] - R[1, 2]) / s,
                (R[0, 2] - R[2, 0]) / s,
                (R[1, 0] - R[0, 1]) / s,
            ]
        )
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = np.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k]) * 2.0
        q = np.zeros(4)
        q[0] = (R[k, j] - R[j, k]) / s
        q[i + 1] = 0.25 * s
        q[j + 1] = (R[i, j] + R[j, i]) / s
        q[k + 1] = (R[i, k] + R[k, i]) / s
    return q / np.linalg.norm(q)


def set_pose(model, data, q, frame):
    import mujoco

    for name, value in zip(JOINTS, q):
        data.qpos[model.joint(name).qposadr] = value
    data.mocap_pos[0] = frame[:3, 3]
    data.mocap_quat[0] = quat_from_matrix(frame[:3, :3])
    mujoco.mj_forward(model, data)


def save(renderer, model, data, path):
    import mujoco

    cam = mujoco.MjvCamera()
    cam.lookat[:] = [-0.05, -0.05, 0.35]
    cam.distance = 1.7
    cam.azimuth = 140
    cam.elevation = -22
    renderer.update_scene(data, camera=cam)
    img = renderer.render()
    from PIL import Image

    Image.fromarray(img).save(path)
    print(path, img.shape)


def main():
    import mujoco

    model = mujoco.MjModel.from_xml_path(str(ROOT / "model" / "scene.xml"))
    data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, height=720, width=960)
    frames = ROOT / "frames"
    frames.mkdir(exist_ok=True)

    target = forward(Q_GOAL)
    set_pose(model, data, Q_START, forward(Q_START))
    save(renderer, model, data, frames / "ik_start.png")

    set_pose(model, data, Q_GOAL, target)
    save(renderer, model, data, frames / "fk.png")

    q_ik, _ = solve(Q_START, target)
    set_pose(model, data, q_ik, target)
    save(renderer, model, data, frames / "ik.png")


if __name__ == "__main__":
    main()
