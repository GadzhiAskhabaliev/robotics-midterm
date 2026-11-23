# Parallel Gripper

Native SOLIDWORKS parallel gripper: gear-driven crank–link fingers, 1:1 gear mate, parallelogram keeps the jaws parallel.

## Open in SolidWorks

1. Open `solidworks/Assem1.SLDASM` (keep all `PG_*.SLDPRT` in the same folder — do not rename).
2. Show motion:
   - Configuration Manager → Open / Half / Grip_62mm (if present), or
   - Macro → Run `macros/Demo_OpenClose_Live.swb`, or
   - drag the drive gear if DOF is free.

## Layout

| Path | Contents |
|------|----------|
| `solidworks/` | `Assem1.SLDASM` + native `PG_*.SLDPRT` |
| `geometry/parts/` | Per-part STEP |
| `geometry/poses/` | Open / closed STEP & STL |
| `frames/` | `gripper.gif`, open/closed PNGs, `Grasp_MuJoCo.mp4` (physics reference) |
| `macros/` | Live open–close demo, Unfix, Motion Study helper |
| `out/` | CadQuery tutorial-style STEP/STL (reference) |
| `BOM.txt` | Part list |

## Mechanism

Two opposing gear arms + follower links form a parallelogram. Opening is controlled by gear rotation / angle mate (or saved configurations), not a failed builder “jaw opening dimension” step.
