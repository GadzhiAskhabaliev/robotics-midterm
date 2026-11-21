# UR3 forward and inverse kinematics

Modified Denavit-Hartenberg parameters from the task sheet.

| i | alpha(i-1) | a(i-1) | d(i) | theta(i) |
| --- | --- | --- | --- | --- |
| 1 | 0 | 0 | 0.152 | q1 |
| 2 | pi/2 | 0 | 0 | q2 |
| 3 | 0 | -0.244 | 0 | q3 |
| 4 | 0 | -0.213 | 0.131 | q4 |
| 5 | pi/2 | 0 | 0.085 | q5 |
| 6 | -pi/2 | 0 | 0.092 | q6 |

`fk.py` multiplies the six link transforms. `forward(q)` returns the 4x4 pose of frame 6.

`ik.py` solves a full pose with damped least squares. The residual is the position difference plus the rotation vector between the target orientation and the current orientation. The Jacobian is a central difference of that residual. One step is

`dq = J^T (J J^T + 0.05^2 I)^{-1} e`

and the joint step is limited to 0.2 rad.

## Model

`model/scene.xml` is the UR3 CB3 with a Robotiq 2F-85, taken from https://github.com/miguelaltamirano/ur3-ik-playground. Source and licenses are in `model/SOURCE.txt`. The six hinge angles are the same `q` used by `fk.py` and `ik.py`. The red, green and blue axes are frame 6 of this DH chain. The gripper is mounted past that frame, so the fingertips are not the DH origin.

## Check

```bash
python3 check.py
python3 build_scene.py
python3 render.py
```

`check.py` needs NumPy. `build_scene.py` rewrites the OBJ meshes and `scene.xml` from `model/meshes.bin`. `render.py` needs MuJoCo and writes `frames/fk.png`, `frames/ik_start.png` and `frames/ik.png`. The start pose and the IK result are different joint vectors. The IK result matches the FK pose of the goal.

The recorded check is `results/check.txt`.
