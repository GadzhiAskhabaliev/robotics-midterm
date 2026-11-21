# UR3 forward kinematics

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

The pose in `frames/fk.png` and `results/forward.txt` is `forward([0.4, -1.0, 1.1, -1.3, 1.2, 0.5])`.

## Model

`model/scene.xml` is the UR3 CB3 with a Robotiq 2F-85, taken from https://github.com/miguelaltamirano/ur3-ik-playground. Source and licenses are in `model/SOURCE.txt`. The red, green and blue axes are frame 6 of this DH chain. The gripper is mounted past that frame, so the fingertips are not the DH origin.

## Run

```bash
python3 -c "from fk import forward; print(forward([0.4, -1.0, 1.1, -1.3, 1.2, 0.5]))"
```

NumPy is required. `build_scene.py` rewrites the OBJ meshes and `scene.xml` from `model/meshes.bin`.
