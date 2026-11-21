# Parallel gripper

Seven parts, millimetres: Holder, Servo, ToothGear, CrankGear, Link, Finger, Pin. Both gears carry a primary arm toward the jaws; two Links are the secondary bars; the Finger tip follows the hooked profile from the course sketch.

`out/GripperOpen.step` and `out/GripperOpen.stl` are the open pose. `out/GripperClosed.step` and `out/GripperClosed.stl` are the closed pose. Each part is also a separate STEP file in `out/`.

`frames/GripperOpen.png` and `frames/GripperClosed.png` show the assembly in those poses. `frames/GripperOpen_side.png` and `frames/GripperClosed_side.png` are the side views. `frames/gripper.gif` and `frames/gripper_wide.gif` are the jaw motion from open to closed and back; the wide file uses a farther camera. `frames/GripperOpen_wide.png` and `frames/GripperClosed_wide.png` match that view.

`build_gripper.py` writes those files with CadQuery. A STEP file that is already present is left as it is.

`solidworks/` holds the SolidWorks assembly and parts: `Assem1.SLDASM`, `Holder.SLDPRT`, `Servo.SLDPRT`, `ToothGear.SLDPRT`, `CrankGear.SLDPRT`, `Link.SLDPRT`, `Finger.SLDPRT`, `Pin.SLDPRT`. Open `Assem1.SLDASM` from that folder.

`build_gripper.swb` builds the same parts in SolidWorks, saves them as SLDPRT, opens the assembly, and moves the jaws from the open pose to the closed pose. The macro stores the parts in `C:\Temp\Gripper`, or `C:\Users\Public\Gripper`, or `D:\Gripper`.

## Run

```bash
python3 build_gripper.py
```

In SolidWorks: Tools, Macro, Run, and choose `build_gripper.swb`.
