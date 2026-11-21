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
    """Finger from Finger_1: two Ø10 bosses 22 mm apart, 25° arm, hooked jaw."""
    depth = 3
    ang = math.radians(25)
    c, s = math.cos(ang), math.sin(ang)
    nx, ny = -s, c
    x0, y0 = 22.0, 0.0
    # arm length 36 to the outer corner along the centerline
    x1 = x0 + 36 * c
    y1 = y0 + 36 * s
    half = 2.5
    arm = [
        (x0 + nx * half, y0 + ny * half),
        (x1 + nx * half, y1 + ny * half),
        (x1 - nx * half, y1 - ny * half),
        (x0 - nx * half, y0 - ny * half),
    ]
    # hooked jaw: 135° inside, tip flat ~1, grip face toward -ny
    tip_dir_x = c * math.cos(math.radians(45)) - s * math.sin(math.radians(45))
    tip_dir_y = s * math.cos(math.radians(45)) + c * math.sin(math.radians(45))
    # jaw polygon in the plane of the arm
    j0 = (x1 - nx * half, y1 - ny * half)
    j1 = (x1 + nx * half, y1 + ny * half)
    j2 = (j1[0] + 11 * c, j1[1] + 11 * s)
    j3 = (j2[0] + 5.5 * tip_dir_x, j2[1] + 5.5 * tip_dir_y)
    j4 = (j3[0] - nx * 1.0, j3[1] - ny * 1.0)  # tip flat ~1
    j5 = (j0[0] + 10 * c - nx * 0.5, j0[1] + 10 * s - ny * 0.5)
    jaw = [j0, j1, j2, j3, j4, j5]
    body = circle(0, 0, 5, depth)
    body = body.union(rect(0, -5, 22, 5, depth))
    body = body.union(circle(22, 0, 5, depth))
    body = body.union(poly(arm, depth))
    body = body.union(poly(jaw, depth))
    body = body.cut(hole(0, 0, 1.5, depth))
    body = body.cut(hole(22, 0, 1.5, depth))
    return body


def gear_blank(depth):
    return circle(0, 0, 12.1, depth).union(teeth(24, 12.1, 14.65, depth))


def crank():
    """Crank gear: arm along +X toward the jaws (matches Assembly_8 layout)."""
    depth = 3
    body = gear_blank(depth)
    body = body.union(rect(0, -3, 31, 3, depth))
    body = body.union(circle(31, 0, 4, depth))
    body = body.cut(hole(0, 0, 1.5, depth))
    body = body.cut(hole(31, 0, 1.5, depth))
    return body


def tooth():
    """Driven gear with arm (Assembly_8: both gears carry a primary arm toward the jaws)."""
    depth = 3
    body = gear_blank(depth)
    body = body.union(rect(0, -3, 31, 3, depth))
    body = body.union(circle(31, 0, 4, depth))
    body = body.cut(hole(0, 0, 1.5, depth))
    body = body.cut(hole(31, 0, 1.5, depth))
    return body


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


def load(name, builder, force=False):
    path = OUT / f"{name}.step"
    if path.exists() and not force:
        return cq.importers.importStep(str(path))
    return builder()


def assembly(closed, parts_cache=None):
    """Layout like Assembly_8: two geared arms, two secondary Links, two Fingers."""
    jaw = 1.0 if closed else 0.0
    if parts_cache is None:
        parts_cache = {
            "Holder": load("Holder", holder),
            "Servo": load("Servo", servo),
            "ToothGear": load("ToothGear", tooth, force=True),
            "CrankGear": load("CrankGear", crank, force=True),
            "Link": load("Link", link),
            "Finger": load("Finger", finger, force=True),
            "Pin": load("Pin", pin),
        }
    p = parts_cache
    # opposite rotation, both arms toward +X (jaws)
    crank_ang = -0.40 + 0.80 * jaw
    tooth_ang = -crank_ang
    c_tip_x = 26 + 31 * math.cos(crank_ang)
    c_tip_y = 10 + 31 * math.sin(crank_ang)
    t_tip_x = 10 + 31 * math.cos(tooth_ang)
    t_tip_y = 10 + 31 * math.sin(tooth_ang)
    # finger orientation: continue past the arm tip
    low_ang = crank_ang - 0.05
    up_ang = tooth_ang + 0.05
    # finger hole at 22 mm (second pivot)
    low_h2_x = c_tip_x + 22 * math.cos(low_ang)
    low_h2_y = c_tip_y + 22 * math.sin(low_ang)
    up_h2_x = t_tip_x + 22 * math.cos(up_ang)
    up_h2_y = t_tip_y + 22 * math.sin(up_ang)
    # secondary Links: holder ground hole (10,-4) -> lower h2; and a link near upper side
    # upper secondary: from a point near holder lobe toward upper h2
    # use link length 31: place so one end is at h2 and the other toward the holder
    low_link_ang = math.atan2(low_h2_y - (-4), low_h2_x - 10)
    # upper secondary pivots about (10, 10) offset — use holder-top style: from (26, 10) no;
    # Assembly_8: each secondary link joins primary-arm elbow area to the claw.
    # Place upper link from tooth tip toward upper finger hole 2 (along finger).
    up_link_ang = up_ang
    parts = [
        place(p["Holder"], 0, 0, 0),
        place(p["Servo"], 4, -2, -20),
        place(p["ToothGear"], 10, 10, 4, tooth_ang),
        place(p["CrankGear"], 26, 10, 4, crank_ang),
        # secondary dogbone links
        place(p["Link"], 10, -4, 7, low_link_ang),
        place(p["Link"], t_tip_x, t_tip_y, 8, up_link_ang),
        place(p["Finger"], c_tip_x, c_tip_y, 4, low_ang, False),
        place(p["Finger"], t_tip_x, t_tip_y, 8, up_ang, True),
        place(p["Pin"], 10, 10, 2),
        place(p["Pin"], 26, 10, 2),
        place(p["Pin"], c_tip_x, c_tip_y, 1),
        place(p["Pin"], t_tip_x, t_tip_y, 1),
        place(p["Pin"], 10, -4, 1),
        place(p["Pin"], low_h2_x, low_h2_y, 2),
        place(p["Pin"], up_h2_x, up_h2_y, 2),
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
    rewrite = {"Finger", "CrankGear", "ToothGear"}
    for name, builder in named.items():
        path = OUT / f"{name}.step"
        if path.exists() and name not in rewrite:
            print(name, "kept")
            continue
        cq.exporters.export(builder(), str(path))
        print(name, "wrote")
    cache = {n: load(n, b, force=(n in rewrite)) for n, b in named.items()}
    opened = assembly(False, cache)
    closed = assembly(True, cache)
    cq.exporters.export(opened, str(OUT / "GripperOpen.step"))
    cq.exporters.export(closed, str(OUT / "GripperClosed.step"))
    cq.exporters.export(opened, str(OUT / "GripperOpen.stl"))
    cq.exporters.export(closed, str(OUT / "GripperClosed.stl"))
    print("assembly")


if __name__ == "__main__":
    main()
