"""Parallel gripper from the course tutorial drawings. Millimetres.

Four-bar per jaw (Assembly_8 / video):
  - each gear has a 31 mm drive arm
  - arm tip pins to Finger hole at 22 mm (toward the pad)
  - Link (31 mm) pins Finger hole at 0 to the opposite gear centre
  - gears 1:1 mesh so tooth angle = -crank angle
"""

import math
from pathlib import Path

import cadquery as cq

OUT = Path(__file__).resolve().parent / "out"
OUT.mkdir(exist_ok=True)

LINK_LEN = 31.0
FINGER_SPAN = 22.0
CRANK = (26.0, 10.0)  # right gear centre on Holder
TOOTH = (10.0, 10.0)  # left gear centre on Holder


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
    # Tutorial Finger_1: holes 22 mm apart, 155° interior → arm at +25°, pad beyond.
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
    # Tutorial Crank Gear_3: arm 31 mm. Built along +X; assembly rotates it.
    depth = 3
    body = gear_blank(depth)
    body = body.union(rect(0, -3, 31, 3, depth))
    body = body.union(circle(31, 0, 4, depth))
    body = body.cut(hole(0, 0, 1.5, depth))
    body = body.cut(hole(31, 0, 1.5, depth))
    return body


def tooth():
    # Same blank + arm as crank (tutorial gear_2 also has an arm).
    return crank()


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
    """SG90-ish body under the holder (not a floating cube)."""
    body = rect(-11.5, -6, 11.5, 16, 22)
    ears = rect(-16, -1, -11.5, 11, 2).translate((0, 0, 18))
    ears = ears.union(rect(11.5, -1, 16, 11, 2).translate((0, 0, 18)))
    body = body.union(ears)
    # horn boss under drive gear axis
    body = body.union(circle(0, 0, 3.5, 4).translate((0, 0, 22)))
    return body


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


def _dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _circle_intersect(p1, r1, p2, r2):
    x1, y1 = p1
    x2, y2 = p2
    d = _dist(p1, p2)
    if d > r1 + r2 or d < abs(r1 - r2) or d == 0:
        return []
    a = (r1 * r1 - r2 * r2 + d * d) / (2 * d)
    h = math.sqrt(max(0.0, r1 * r1 - a * a))
    xm = x1 + a * (x2 - x1) / d
    ym = y1 + a * (y2 - y1) / d
    return [
        (xm + h * (y2 - y1) / d, ym - h * (x2 - x1) / d),
        (xm - h * (y2 - y1) / d, ym + h * (x2 - x1) / d),
    ]


def arm_tip(center, ang):
    return (center[0] + LINK_LEN * math.cos(ang), center[1] + LINK_LEN * math.sin(ang))


def jaw_arm_angles(t):
    """t=0 open, t=1 closed. Equal ±angles → arm tips mirrored about y=10."""
    open_deg, closed_deg = 55.0, 32.0
    au = math.radians(open_deg + (closed_deg - open_deg) * t)
    return au, -au


def finger_hole0(h22, link_pivot, prefer_phi=0.0):
    """Exact four-bar: |h0-h22|=22 and |h0-pivot|=31. Finger points outward."""
    cands = _circle_intersect(h22, FINGER_SPAN, link_pivot, LINK_LEN)
    scored = []
    for h0 in cands:
        phi = math.atan2(h22[1] - h0[1], h22[0] - h0[0])
        if math.cos(phi) < 0.15:
            continue
        err = abs(math.atan2(math.sin(phi - prefer_phi), math.cos(phi - prefer_phi)))
        scored.append((err, h0, phi))
    if not scored:
        raise RuntimeError(f"no connected finger pose for h22={h22} pivot={link_pivot}")
    scored.sort()
    return scored[0][1], scored[0][2]


def pose_params(t):
    """Connected four-bar (Assembly_8):

    gear arm tip == Finger hole22 (pinned)
    Link from opposite gear centre == Finger hole0 (pinned, length 31)
    """
    au, al = jaw_arm_angles(t)
    h22u = arm_tip(CRANK, au)
    h22l = arm_tip(TOOTH, al)
    h0u, phiu = finger_hole0(h22u, TOOTH, prefer_phi=0.0)
    h0l, phil = finger_hole0(h22l, CRANK, prefer_phi=0.0)
    # Link origin at gear centre, far hole lands exactly on h0 (length 31).
    link_u = math.atan2(h0u[1] - TOOTH[1], h0u[0] - TOOTH[0])
    link_l = math.atan2(h0l[1] - CRANK[1], h0l[0] - CRANK[0])
    return {
        "au": au,
        "al": al,
        "h22u": h22u,
        "h22l": h22l,
        "h0u": h0u,
        "h0l": h0l,
        "phiu": phiu,
        "phil": phil,
        "link_u": link_u,
        "link_l": link_l,
    }


def assembly(closed):
    t = 1.0 if closed else 0.0
    p = pose_params(t)
    # Finger local origin is hole0; hole22 is at +22 along local +X after rotation by phi.
    parts = [
        place(load("Holder", holder), 0, 0, 0),
        place(load("Servo", servo), 4, -2, -20),
        place(load("ToothGear", tooth, force=True), TOOTH[0], TOOTH[1], 4, p["al"]),
        place(load("CrankGear", crank, force=True), CRANK[0], CRANK[1], 4, p["au"]),
        # Upper link: Tooth centre → upper hole0
        place(load("Link", link), TOOTH[0], TOOTH[1], 8, p["link_u"]),
        # Lower link: Crank centre → lower hole0
        place(load("Link", link), CRANK[0], CRANK[1], 8, p["link_l"]),
        # Lower finger mirrored so pad faces +Y; upper pad faces -Y.
        # Same Z for both jaws so one does not look "lower" from stacking.
        place(load("Finger", finger), p["h0l"][0], p["h0l"][1], 6, p["phil"], True),
        place(load("Finger", finger), p["h0u"][0], p["h0u"][1], 6, p["phiu"], False),
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
    force = {"CrankGear", "ToothGear"}  # arm direction changed to +X
    for name, builder in named.items():
        path = OUT / f"{name}.step"
        if path.exists() and name not in force:
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
    po, pc = pose_params(0), pose_params(1)
    print("open  au", round(math.degrees(po["au"]), 1), "phiu", round(math.degrees(po["phiu"]), 1))
    print("closed au", round(math.degrees(pc["au"]), 1), "phiu", round(math.degrees(pc["phiu"]), 1))


if __name__ == "__main__":
    main()
