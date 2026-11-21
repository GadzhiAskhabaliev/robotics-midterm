"""Parallel gripper from the course drawings. Millimetres. Writes STEP and STL."""

import math
from pathlib import Path

import cadquery as cq

OUT = Path(__file__).resolve().parent / "out"
OUT.mkdir(exist_ok=True)


def circle(x, y, r, depth):
    return cq.Workplane("XY").center(x, y).circle(r).extrude(depth)


def rect(x1, y1, x2, y2, depth):
    return (
        cq.Workplane("XY")
        .moveTo(x1, y1)
        .lineTo(x2, y1)
        .lineTo(x2, y2)
        .lineTo(x1, y2)
        .close()
        .extrude(depth)
    )


def poly(pts, depth):
    return cq.Workplane("XY").polyline(pts).close().extrude(depth)


def hole(x, y, r, depth):
    return circle(x, y, r, depth + 2).translate((0, 0, -1))


def teeth(n, r_root, r_tip, depth):
    step = 2 * math.pi / n
    half = step * 0.22
    body = None
    for i in range(n):
        ang = i * step
        pts = [
            (r_root * math.cos(ang - half), r_root * math.sin(ang - half)),
            (r_tip * math.cos(ang - half * 0.45), r_tip * math.sin(ang - half * 0.45)),
            (r_tip * math.cos(ang + half * 0.45), r_tip * math.sin(ang + half * 0.45)),
            (r_root * math.cos(ang + half), r_root * math.sin(ang + half)),
        ]
        tooth = poly(pts, depth)
        body = tooth if body is None else body.union(tooth)
    return body


def link():
    depth = 3
    body = circle(0, 0, 3.5, depth)
    body = body.union(circle(31, 0, 3.5, depth))
    body = body.union(rect(0, -1.5, 31, 1.5, depth))
    body = body.cut(hole(0, 0, 1.5, depth))
    body = body.cut(hole(31, 0, 1.5, depth))
    return body


def finger():
    depth = 3
    ang = math.radians(25)
    c, s = math.cos(ang), math.sin(ang)
    nx, ny = -s, c
    x0, y0 = 22.0, 0.0
    x1 = x0 + 36 * c
    y1 = y0 + 36 * s
    arm = [
        (x0 + nx * 2.5, y0 + ny * 2.5),
        (x1 + nx * 2.5, y1 + ny * 2.5),
        (x1 - nx * 2.5, y1 - ny * 2.5),
        (x0 - nx * 2.5, y0 - ny * 2.5),
    ]
    pad = [
        (x1, y1),
        (x1 + 16 * c, y1 + 16 * s),
        (x1 + 16 * c - nx * 12, y1 + 16 * s - ny * 12),
        (x1 - nx * 12, y1 - ny * 12),
    ]
    body = circle(0, 0, 5, depth)
    body = body.union(rect(0, -5, 22, 5, depth))
    body = body.union(circle(22, 0, 5, depth))
    body = body.union(poly(arm, depth))
    body = body.union(poly(pad, depth))
    body = body.cut(hole(0, 0, 1.5, depth))
    body = body.cut(hole(22, 0, 1.5, depth))
    return body


def gear_blank(depth):
    return circle(0, 0, 12.1, depth).union(teeth(24, 12.1, 14.65, depth))


def crank():
    depth = 3
    body = gear_blank(depth)
    body = body.union(rect(-31, -3, 0, 3, depth))
    body = body.union(circle(-31, 0, 4, depth))
    body = body.cut(hole(0, 0, 1.5, depth))
    body = body.cut(hole(-31, 0, 1.5, depth))
    return body


def tooth():
    depth = 3
    return gear_blank(depth).cut(hole(0, 0, 1.5, depth))


def holder():
    depth = 4
    outline = [
        (0, -10),
        (10, -10),
        (26, 0),
        (40, 0),
        (50, 10),
        (40, 15),
        (10, 15),
        (0, 5),
    ]
    body = poly(outline, depth)
    body = body.union(circle(50, 10, 10, depth))
    body = body.cut(hole(10, 10, 1.5, depth))
    body = body.cut(hole(26, 10, 1.5, depth))
    body = body.cut(hole(10, -4, 1.5, depth))
    slot = circle(30, 5, 6, depth).union(circle(42, 5, 6, depth)).union(rect(30, -1, 42, 11, depth))
    body = body.cut(slot.translate((0, 0, -1)))
    return body


def pin():
    return circle(0, 0, 1.45, 12)


def servo():
    return rect(-10, -6, 12, 16, 20)


def place(shape, x, y, z, ang=0.0, mirror=False):
    solid = shape.val() if isinstance(shape, cq.Workplane) else shape
    if mirror:
        solid = solid.mirror("XZ")
    loc = cq.Location(cq.Vector(x, y, z), cq.Vector(0, 0, 1), math.degrees(ang))
    return solid.located(loc)


def load(name, builder):
    path = OUT / f"{name}.step"
    if path.exists():
        return cq.importers.importStep(str(path))
    return builder()


def assembly(closed):
    jaw = 0.55 if closed else 0.0
    parts = [
        place(load("Holder", holder), 0, 0, 0),
        place(load("Servo", servo), 4, -2, -20),
        place(load("ToothGear", tooth), 10, 10, 4),
        place(load("CrankGear", crank), 26, 10, 4),
        place(load("Link", link), 26, 10, 8, 0),
        place(load("Link", link), 10, -4, 8, 1.2),
        place(load("Finger", finger), 55, -12, 4, -0.45 + jaw, False),
        place(load("Finger", finger), 55, 28, 8, 0.45 - jaw, True),
        place(load("Pin", pin), 10, 10, 2),
        place(load("Pin", pin), 26, 10, 2),
        place(load("Pin", pin), 55, -12, 2),
        place(load("Pin", pin), 55, 28, 2),
    ]
    body = parts[0]
    for part in parts[1:]:
        body = body.fuse(part)
    return body


def main():
    named = {
        "Link": link,
        "Finger": finger,
        "CrankGear": crank,
        "ToothGear": tooth,
        "Holder": holder,
        "Pin": pin,
        "Servo": servo,
    }
    for name, builder in named.items():
        path = OUT / f"{name}.step"
        if path.exists():
            print(name, "kept")
            continue
        cq.exporters.export(builder(), str(path))
        print(name)
    opened = assembly(False)
    closed = assembly(True)
    cq.exporters.export(opened, str(OUT / "GripperOpen.step"))
    cq.exporters.export(closed, str(OUT / "GripperClosed.step"))
    cq.exporters.export(opened, str(OUT / "GripperOpen.stl"))
    cq.exporters.export(closed, str(OUT / "GripperClosed.stl"))
    print("assembly")


if __name__ == "__main__":
    main()
