"""Unpack the course mesh pack and write a MuJoCo scene for the same links."""

import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
MODEL = ROOT / "model"
MESH_DIR = MODEL / "meshes"


def rpy_matrix(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def matrix_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([w, x, y, z], dtype=float)
    return q / np.linalg.norm(q)


def fmt(values):
    return " ".join("%.8g" % float(v) for v in values)


def write_obj(path, vertices, indices):
    lines = []
    for v in vertices:
        lines.append("v %.8g %.8g %.8g" % (v[0], v[1], v[2]))
    for i in range(0, len(indices), 3):
        a, b, c = indices[i] + 1, indices[i + 1] + 1, indices[i + 2] + 1
        if a == b or b == c or a == c:
            continue
        lines.append("f %d %d %d" % (a, b, c))
    path.write_text("\n".join(lines) + "\n")


def unpack():
    spec = json.loads((MODEL / "robot.json").read_text())
    raw = (MODEL / "meshes.bin").read_bytes()
    MESH_DIR.mkdir(exist_ok=True)
    for name, mesh in spec["meshes"].items():
        verts = np.frombuffer(
            raw,
            dtype="<f4",
            count=mesh["vertexCount"] * 3,
            offset=mesh["vertexOffset"],
        ).reshape(-1, 3).copy()
        idx = np.frombuffer(
            raw,
            dtype="<u4",
            count=mesh["indexCount"],
            offset=mesh["indexOffset"],
        ).copy()
        write_obj(MESH_DIR / (name + ".obj"), verts, idx)
    return spec


def scene_xml(spec):
    visuals = {link["name"]: link.get("visuals") or [] for link in spec["links"]}
    children = {}
    for joint in spec["joints"]:
        children.setdefault(joint["parent"], []).append(joint)

    lines = [
        '<mujoco model="ur3_cb3_robotiq_2f85">',
        '  <compiler angle="radian" meshdir="meshes"/>',
        '  <option gravity="0 0 0"/>',
        "  <visual>",
        '    <global offwidth="960" offheight="720"/>',
        "  </visual>",
        "  <asset>",
    ]
    for name in spec["meshes"]:
        lines.append('    <mesh name="%s" file="%s.obj"/>' % (name, name))
    lines += [
        "  </asset>",
        "  <worldbody>",
        '    <light pos="0.6 -0.8 1.4" dir="-0.3 0.5 -1"/>',
        '    <geom type="plane" size="1 1 0.01" pos="0 0 -0.002" rgba="0.93 0.93 0.93 1"/>',
        '    <body name="dh_frame" mocap="true" pos="0.3 0.2 0.4">',
        '      <geom type="capsule" fromto="0 0 0 0.07 0 0" size="0.003" rgba="0.9 0.1 0.1 1" contype="0" conaffinity="0"/>',
        '      <geom type="capsule" fromto="0 0 0 0 0.07 0" size="0.003" rgba="0.1 0.7 0.1 1" contype="0" conaffinity="0"/>',
        '      <geom type="capsule" fromto="0 0 0 0 0 0.07" size="0.003" rgba="0.1 0.2 0.9 1" contype="0" conaffinity="0"/>',
        "    </body>",
    ]

    def emit(link, indent):
        pad = "  " * indent
        for vis in visuals.get(link, []):
            quat = matrix_to_quat(rpy_matrix(*vis["rpy"]))
            rgba = fmt(vis["rgba"])
            lines.append(
                '%s<geom type="mesh" mesh="%s" pos="%s" quat="%s" rgba="%s" contype="0" conaffinity="0"/>'
                % (pad, vis["mesh"], fmt(vis["xyz"]), fmt(quat), rgba)
            )
        for joint in children.get(link, []):
            quat = matrix_to_quat(rpy_matrix(*joint["rpy"]))
            lines.append(
                '%s<body name="%s" pos="%s" quat="%s">'
                % (pad, joint["child"], fmt(joint["xyz"]), fmt(quat))
            )
            if joint["type"] == "revolute":
                lines.append(
                    '%s  <joint name="%s" type="hinge" axis="%s" limited="false"/>'
                    % (pad, joint["name"], fmt(joint["axis"]))
                )
            emit(joint["child"], indent + 1)
            lines.append("%s</body>" % pad)

    emit("world", 2)
    lines += ["  </worldbody>", "</mujoco>", ""]
    return "\n".join(lines)


def main():
    spec = unpack()
    xml = scene_xml(spec)
    (MODEL / "scene.xml").write_text(xml)
    print("meshes", len(spec["meshes"]))
    print("scene", MODEL / "scene.xml")


if __name__ == "__main__":
    main()
