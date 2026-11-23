"""Render open/closed PNGs and open→closed→open GIF from tutorial kinematics."""

import math
from pathlib import Path

import cadquery as cq
from PIL import Image
from vtkmodules.vtkIOGeometry import vtkSTLReader
from vtkmodules.vtkIOImage import vtkPNGWriter
from vtkmodules.vtkRenderingCore import (
    vtkActor,
    vtkLight,
    vtkPolyDataMapper,
    vtkRenderer,
    vtkRenderWindow,
    vtkWindowToImageFilter,
)

from build_gripper import OUT, assembly, pose_params

ROOT = Path(__file__).resolve().parent
FRAMES = ROOT / "frames"
FRAMES.mkdir(exist_ok=True)


def render_stl(stl_path, png_path, size=(1000, 720), pos=(110, -130, 80), focal=(40, 8, 4), scale=None):
    reader = vtkSTLReader()
    reader.SetFileName(str(stl_path))
    reader.Update()
    mapper = vtkPolyDataMapper()
    mapper.SetInputConnection(reader.GetOutputPort())
    actor = vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetColor(0.72, 0.74, 0.78)
    actor.GetProperty().SetSpecular(0.35)
    actor.GetProperty().SetSpecularPower(28)

    ren = vtkRenderer()
    ren.AddActor(actor)
    ren.SetBackground(0.92, 0.93, 0.95)
    light = vtkLight()
    light.SetPosition(90, 140, 160)
    light.SetFocalPoint(*focal)
    light.SetIntensity(1.15)
    ren.AddLight(light)

    rw = vtkRenderWindow()
    rw.SetOffScreenRendering(1)
    rw.AddRenderer(ren)
    rw.SetSize(*size)
    cam = ren.GetActiveCamera()
    ren.ResetCamera()
    cam.SetPosition(*pos)
    cam.SetFocalPoint(*focal)
    cam.SetViewUp(0, 0, 1)
    if scale is not None:
        cam.ParallelProjectionOn()
        cam.SetParallelScale(scale)
    ren.ResetCameraClippingRange()
    rw.Render()

    w2i = vtkWindowToImageFilter()
    w2i.SetInput(rw)
    w2i.Update()
    writer = vtkPNGWriter()
    writer.SetFileName(str(png_path))
    writer.SetInputConnection(w2i.GetOutputPort())
    writer.Write()


def stl_pose(t, path):
    # Rebuild fused solid at parameter t without going through assembly(closed) bool only
    from build_gripper import (
        CRANK,
        TOOTH,
        crank,
        finger,
        holder,
        link,
        load,
        pin,
        place,
        servo,
        tooth,
    )

    p = pose_params(t)
    parts = [
        place(load("Holder", holder), 0, 0, 0),
        place(load("Servo", servo), 4, -2, -20),
        place(load("ToothGear", tooth), TOOTH[0], TOOTH[1], 4, p["al"]),
        place(load("CrankGear", crank), CRANK[0], CRANK[1], 4, p["au"]),
        place(load("Link", link), TOOTH[0], TOOTH[1], 8, p["link_u"]),
        place(load("Link", link), CRANK[0], CRANK[1], 8, p["link_l"]),
        place(load("Finger", finger), p["h0l"][0], p["h0l"][1], 4, p["phil"], True),
        place(load("Finger", finger), p["h0u"][0], p["h0u"][1], 8, p["phiu"], False),
        place(load("Pin", pin), TOOTH[0], TOOTH[1], 2),
        place(load("Pin", pin), CRANK[0], CRANK[1], 2),
        place(load("Pin", pin), p["h0l"][0], p["h0l"][1], 2),
        place(load("Pin", pin), p["h0u"][0], p["h0u"][1], 2),
        place(load("Pin", pin), p["h22l"][0], p["h22l"][1], 2),
        place(load("Pin", pin), p["h22u"][0], p["h22u"][1], 2),
    ]
    body = parts[0]
    for part in parts[1:]:
        body = body.fuse(part)
    cq.exporters.export(body, str(path))


def main():
    cq.exporters.export(assembly(False), str(OUT / "GripperOpen.stl"))
    cq.exporters.export(assembly(True), str(OUT / "GripperClosed.stl"))
    cq.exporters.export(assembly(False), str(OUT / "GripperOpen.step"))
    cq.exporters.export(assembly(True), str(OUT / "GripperClosed.step"))

    render_stl(OUT / "GripperOpen.stl", FRAMES / "GripperOpen.png", scale=48)
    render_stl(OUT / "GripperClosed.stl", FRAMES / "GripperClosed.png", scale=48)
    render_stl(OUT / "GripperOpen.stl", FRAMES / "GripperOpen_wide.png", scale=62)
    render_stl(OUT / "GripperClosed.stl", FRAMES / "GripperClosed_wide.png", scale=62)
    render_stl(
        OUT / "GripperOpen.stl",
        FRAMES / "GripperOpen_side.png",
        pos=(40, -180, 25),
        focal=(40, 5, 4),
        scale=50,
    )
    render_stl(
        OUT / "GripperClosed.stl",
        FRAMES / "GripperClosed_side.png",
        pos=(40, -180, 25),
        focal=(40, 5, 4),
        scale=50,
    )

    frames = []
    steps = 12
    seq = list(range(steps + 1)) + list(range(steps - 1, -1, -1))
    tmp = OUT / "_frame.stl"
    for i, s in enumerate(seq):
        t = s / steps
        stl_pose(t, tmp)
        png = FRAMES / f"_g{i:02d}.png"
        render_stl(tmp, png, size=(800, 560), scale=48)
        frames.append(Image.open(png).convert("P", palette=Image.ADAPTIVE))
        print("frame", i, "t=", round(t, 3), "au", round(math.degrees(pose_params(t)["au"]), 1))

    frames[0].save(
        FRAMES / "gripper.gif",
        save_all=True,
        append_images=frames[1:],
        duration=100,
        loop=0,
        optimize=True,
    )
    frames_w = []
    for i, s in enumerate(seq):
        t = s / steps
        stl_pose(t, tmp)
        png = FRAMES / f"_gw{i:02d}.png"
        render_stl(tmp, png, size=(800, 560), scale=62)
        frames_w.append(Image.open(png).convert("P", palette=Image.ADAPTIVE))
    frames_w[0].save(
        FRAMES / "gripper_wide.gif",
        save_all=True,
        append_images=frames_w[1:],
        duration=100,
        loop=0,
        optimize=True,
    )
    for p in FRAMES.glob("_g*.png"):
        p.unlink()
    tmp.unlink(missing_ok=True)
    print("done", FRAMES / "gripper.gif")


if __name__ == "__main__":
    main()
