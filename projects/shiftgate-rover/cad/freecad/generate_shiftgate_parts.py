# -*- coding: utf-8 -*-
"""FreeCAD headless generator for ShiftGate Rover printed parts.

This script is meant to be run with FreeCAD's Python, not CPython:

    freecadcmd generate_shiftgate_parts.py
    # or:
    FreeCADCmd generate_shiftgate_parts.py

It produces the same part names as the CadQuery generator in
../cadquery/generate_parts.py.  The CadQuery script is what was actually
used to export the STL/STEP files shipped in this repository (verified
watertight with trimesh).  Use this file if your lab already runs FreeCAD
and you want to tweak a dimension in the GUI afterwards.

If you change a dimension, keep it in sync with
software/shiftgate_core/shiftgate_core/params.py and the CadQuery script.
"""

from __future__ import annotations

import json
import math
import os
import sys
from pathlib import Path

# FreeCAD's interpreter inserts its own modules on sys.path.
try:
    import FreeCAD
    import Mesh
    import Part
except ImportError as exc:  # pragma: no cover
    sys.stderr.write(
        "This script must be run with FreeCADCmd / freecadcmd, not system Python.\n"
        "Alternatively, use the CadQuery generator: ../cadquery/generate_parts.py\n"
    )
    raise SystemExit(2) from exc

# --- parameters (mm), mirrored from the CadQuery script ---
CHASSIS_L, CHASSIS_W, PLATE_T, SIDE_H = 118.0, 90.0, 3.0, 22.0
WHEEL_R, WHEEL_W = 24.0, 8.0
SHAFT_D, SHAFT_FLAT = 3.0, 0.5
N20_FACE_X, N20_FACE_Z = 12.2, 10.2
AXLE_X = 28.0
CELL, WALL_T, WALL_H, FOAM_T = 180.0, 4.0, 50.0, 5.0
M3, M2 = 3.2, 2.2


def box(x, y, z, cx, cy, cz):
    """Axis-aligned box whose *centre* is (cx,cy,cz)."""
    sh = Part.makeBox(x, y, z)
    sh.translate(FreeCAD.Vector(cx - x / 2.0, cy - y / 2.0, cz - z / 2.0))
    return sh


def cyl(r, h, cx, cy, cz, axis="z"):
    sh = Part.makeCylinder(r, h)
    if axis == "x":
        sh.rotate(FreeCAD.Vector(0, 0, 0), FreeCAD.Vector(0, 1, 0), 90)
        sh.translate(FreeCAD.Vector(cx, cy, cz - 0))  # after rotate, height along -X? 
        # Recreate more carefully:
        sh = Part.makeCylinder(r, h, FreeCAD.Vector(cx - h / 2.0, cy, cz), FreeCAD.Vector(1, 0, 0))
    elif axis == "y":
        sh = Part.makeCylinder(r, h, FreeCAD.Vector(cx, cy - h / 2.0, cz), FreeCAD.Vector(0, 1, 0))
    else:
        sh = Part.makeCylinder(r, h, FreeCAD.Vector(cx, cy, cz), FreeCAD.Vector(0, 0, 1))
    return sh


def fuse(shapes):
    acc = shapes[0]
    for s in shapes[1:]:
        acc = acc.fuse(s)
    return acc


def cut(base, tools):
    acc = base
    for t in tools:
        acc = acc.cut(t)
    return acc


def wheel():
    rim = cyl(WHEEL_R, WHEEL_W, 0, 0, 0)
    # groove
    groove = Part.makeCylinder(WHEEL_R + 0.2, 2.2, FreeCAD.Vector(0, 0, (WHEEL_W - 2.2) / 2.0))
    groove = groove.cut(Part.makeCylinder(WHEEL_R - 0.8, 2.2, FreeCAD.Vector(0, 0, (WHEEL_W - 2.2) / 2.0)))
    rim = rim.cut(groove)
    hub = Part.makeCylinder(7.0, WHEEL_W + 2.0, FreeCAD.Vector(0, 0, -1.0))
    part = rim.fuse(hub)
    # D-shaft: cylinder minus a slab
    bore = Part.makeCylinder(SHAFT_D / 2.0 + 0.12, WHEEL_W + 4, FreeCAD.Vector(0, 0, -2.0))
    flat_x = (SHAFT_D / 2.0 + 0.12) - SHAFT_FLAT
    slab = box(4.0, 8.0, WHEEL_W + 6, flat_x + 2.0, 0, WHEEL_W / 2.0)
    d = bore.cut(slab)
    part = part.cut(d)
    return part


def chassis_base():
    floor = box(CHASSIS_L, CHASSIS_W, PLATE_T, 0, 0, PLATE_T / 2.0)
    wall_t = 3.0
    y = (CHASSIS_W - wall_t) / 2.0
    walls = fuse(
        [
            box(CHASSIS_L, wall_t, SIDE_H, 0, y, SIDE_H / 2.0),
            box(CHASSIS_L, wall_t, SIDE_H, 0, -y, SIDE_H / 2.0),
            box(3.0, CHASSIS_W, 10.0, -CHASSIS_L / 2.0 + 1.5, 0, 5.0),
        ]
    )
    part = floor.fuse(walls)
    motor_x = -CHASSIS_L / 2.0 + AXLE_X
    shaft_z = 14.0
    pocket = box(N20_FACE_X, CHASSIS_W + 10, N20_FACE_Z, motor_x, 0, shaft_z)
    part = part.cut(pocket)
    # clamp bosses + M3
    extras = []
    holes = []
    for side in (1, -1):
        for dx in (-9.0, 9.0):
            extras.append(box(6.0, 6.0, SIDE_H, motor_x + dx, side * y, SIDE_H / 2.0))
            holes.append(cyl(M3 / 2.0, SIDE_H + 2, motor_x + dx, side * y, -1))
    part = part.fuse(fuse(extras))
    part = cut(part, holes)
    # caster hole
    caster_x = CHASSIS_L / 2.0 - 16.0
    part = part.fuse(cyl(11.0, PLATE_T + 2.0, caster_x, 0, 0))
    part = part.cut(cyl(8.2, PLATE_T + 6, caster_x, 0, -1))
    # front bumper + M3
    part = part.fuse(box(3.0, 40.0, 10.0, CHASSIS_L / 2.0 - 1.5, 0, PLATE_T + 5.0))
    return part.removeSplitter()


def motor_clamp():
    part = box(22.0, 8.0, 3.0, 0, 0, 1.5)
    part = part.cut(box(N20_FACE_X, 9.0, 1.6, 0, 0, 0.8))
    for dx in (-9.0, 9.0):
        part = part.cut(cyl(M3 / 2.0, 5, dx, 0, -1))
    return part


def wall_clip():
    gap, jaw, height, w = FOAM_T + 0.3, 3.0, 18.0, 18.0
    base = box(w, gap + 2 * jaw + 8.0, 3.0, 0, 0, 1.5)
    left = box(w, jaw, height, 0, (gap + jaw) / 2.0, height / 2.0)
    right = box(w, jaw, height, 0, -(gap + jaw) / 2.0, height / 2.0)
    return fuse([base, left, right])


def wall_post():
    w, h, gap = 14.0, WALL_H + 4.0, FOAM_T + 0.3
    body = box(w, w, h, 0, 0, h / 2.0)
    slot_h = WALL_H + 1.0
    for angle in (0, 90, 180, 270):
        slot = box(w + 2, gap, slot_h, 0, 0, 4.0 + slot_h / 2.0)
        slot.rotate(FreeCAD.Vector(0, 0, 0), FreeCAD.Vector(0, 0, 1), angle)
        body = body.cut(slot)
    return body


def wall_segment(handle=False):
    length = CELL - 4.0
    part = box(length, WALL_T, WALL_H, 0, 0, WALL_H / 2.0)
    if handle:
        part = part.fuse(box(16.0, WALL_T + 2.0, 14.0, 0, 0, WALL_H + 7.0))
        part = part.fuse(box(22.0, 2.0, 16.0, 8.0, 0, WALL_H + 16.0))
    return part


def caster_cup():
    body = Part.makeCylinder(5.8, 8.5)
    sphere = Part.makeSphere(4.25, FreeCAD.Vector(0, 0, 4.8))
    part = body.cut(sphere)
    part = part.cut(Part.makeCylinder(3.6, 2.0))
    stem = Part.makeCylinder(4.0, 5.0, FreeCAD.Vector(0, 0, 8.5))
    return part.fuse(stem)


def sensor_bracket_us():
    part = box(53.5, 26.5, 3.0, 0, 0, 1.5)
    for dx in (-13.0, 13.0):
        part = part.cut(cyl(8.25, 5, dx, 0, -1))
    for y in (-12.0, 12.0):
        ear = box(8.0, 8.0, 3.0, -30.75, y, 1.5)
        part = part.fuse(ear)
        part = part.cut(cyl(M3 / 2.0, 5, -30.75, y, -1))
    return part


def sensor_bracket_ir():
    back = box(18.5, 3.0, 16.0, 0, 0, 8.0)
    foot = box(18.5, 16.0, 3.0, 0, 6.5, 1.5)
    part = back.fuse(foot)
    part = part.cut(cyl(M3 / 2.0, 5, 0, 10.0, -1))
    return part


def electronics_cover():
    l, w, h, t = 70.0, 50.0, 18.0, 1.8
    outer = box(l, w, h, 0, 0, h / 2.0)
    inner = box(l - 2 * t, w - 2 * t, h, 0, 0, h / 2.0 + t)
    part = outer.cut(inner)
    part = part.cut(box(12.0, 20.0, 10.0, l / 2.0, 0, 8.0))
    return part


def battery_retainer():
    part = box(69.0, 8.0, 3.0, 0, 0, 1.5)
    for dx in (-32.5, 32.5):
        part = part.fuse(box(3.0, 8.0, 14.0, dx, 0, 7.0))
    return part


def alignment_jig():
    part = box(30.0, 16.0, 30.0, 0, 0, 15.0)
    hole = Part.makeCylinder(1.7, 20, FreeCAD.Vector(-10, 0, 24.0), FreeCAD.Vector(1, 0, 0))
    part = part.cut(hole)
    part = part.cut(box(16.0, 18.0, 8.0, 7.0, 0, 28.0))
    return part


PARTS = {
    "chassis_base": chassis_base,
    "motor_clamp": motor_clamp,
    "wheel": wheel,
    "caster_cup": caster_cup,
    "sensor_bracket_us": sensor_bracket_us,
    "sensor_bracket_ir": sensor_bracket_ir,
    "electronics_cover": electronics_cover,
    "battery_retainer": battery_retainer,
    "wall_clip": wall_clip,
    "wall_post": wall_post,
    "wall_segment": lambda: wall_segment(False),
    "gate_segment": lambda: wall_segment(True),
    "alignment_jig": alignment_jig,
}


def export_all(stl_dir: Path, step_dir: Path) -> None:
    stl_dir.mkdir(parents=True, exist_ok=True)
    step_dir.mkdir(parents=True, exist_ok=True)
    doc = FreeCAD.newDocument("ShiftGate")
    for name, factory in PARTS.items():
        print("  building", name)
        shape = factory()
        obj = doc.addObject("Part::Feature", name)
        obj.Shape = shape
        Mesh.export([obj], str(stl_dir / f"{name}.stl"))
        shape.exportStep(str(step_dir / f"{name}.step"))
    FreeCAD.closeDocument(doc.Name)
    print("done")


if __name__ == "__main__":
    here = Path(__file__).resolve().parent
    stl = Path(sys.argv[1]) if len(sys.argv) > 1 else here.parent / "stl"
    step = Path(sys.argv[2]) if len(sys.argv) > 2 else here.parent / "step"
    print("ShiftGate FreeCAD export")
    export_all(stl, step)
