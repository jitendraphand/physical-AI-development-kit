#!/usr/bin/env python3
"""Parametric CadQuery generator for every ShiftGate Rover printed part.

Units: millimetres.  Run (CadQuery 2.x):

    python generate_parts.py --outdir ../stl --stepdir ../step

The same numbers are documented in ../../software/shiftgate_core/shiftgate_core/params.py
and in PRINT_SETTINGS.md.  If you change a dimension here, change it there too.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import cadquery as cq
from cadquery import exporters

# ---------------------------------------------------------------------------
# Parameters (mm)
# ---------------------------------------------------------------------------

CHASSIS_L = 118.0
CHASSIS_W = 90.0
PLATE_T = 3.0
SIDE_H = 22.0
FLOOR_CLEARANCE = 10.0  # underside of floor above ground

WHEEL_R = 24.0
WHEEL_W = 8.0
WHEEL_HUB_W = 10.0
SHAFT_D = 3.0
SHAFT_FLAT = 0.5  # milling depth; remaining thickness = 3.0 - 0.5
RUBBER_GROOVE_W = 2.2
RUBBER_GROOVE_D = 0.8

# N20 gearbox face is 12 x 10; body is longer along the axle.
N20_FACE_X = 12.2  # along chassis X (fore-aft)
N20_FACE_Z = 10.2  # along vertical
N20_BODY_Y = 16.0  # how deep the pocket swallows the motor
AXLE_X = 28.0  # axle position from the rear face of the chassis
TRACK = 102.0

# 4xAA holder (Robocraze 62 x 58 x 15)
BATT_L = 63.0
BATT_W = 59.0
BATT_H = 16.0

# ESP32 DevKit V1 ~55 x 28
ESP_L = 56.0
ESP_W = 29.0

# HC-SR04
US_L = 45.5
US_W = 20.5
US_T = 15.0
US_HOLE_D = 16.5
US_HOLE_C = 26.0  # transducer centre spacing, approx.

# IR module ~48 x 14
IR_L = 48.5
IR_W = 14.5

# Maze
CELL = 180.0
WALL_T = 4.0
WALL_H = 50.0
FOAM_T = 5.0  # stationery foam-board thickness the clips are sized for
CLIP_W = 18.0
POST_W = 14.0

M3 = 3.2
M2 = 2.2


def _d_shaft_profile(workplane, diameter=SHAFT_D, flat=SHAFT_FLAT, clearance=0.15):
    """2D D-profile slightly oversized for a press/slip fit on a 3 mm D-shaft."""
    r = diameter / 2.0 + clearance
    # Flat is `flat` mm in from the +X side of the circle.
    flat_x = r - flat
    return (
        workplane
        .moveTo(-r, 0)
        .threePointArc((0, r), (flat_x, math.sqrt(max(r * r - flat_x * flat_x, 0.0))))
        .lineTo(flat_x, -math.sqrt(max(r * r - flat_x * flat_x, 0.0)))
        .threePointArc((0, -r), (-r, 0))
        .close()
    )


def wheel() -> cq.Workplane:
    """48 mm wheel with a rubber-band tyre groove and a D-shaft bore."""
    rim = cq.Workplane("XY").circle(WHEEL_R).extrude(WHEEL_W)
    # Recess both faces so the hub is proud and the print sits flat.
    recess = (
        cq.Workplane("XY")
        .workplane(offset=WHEEL_W - 1.6)
        .circle(WHEEL_R - 3.5)
        .extrude(1.7)
    )
    rim = rim.cut(recess)
    recess2 = cq.Workplane("XY").circle(WHEEL_R - 3.5).extrude(1.6)
    rim = rim.cut(recess2)
    # Rubber-band groove around the tread.
    groove = (
        cq.Workplane("XY")
        .workplane(offset=(WHEEL_W - RUBBER_GROOVE_W) / 2.0)
        .circle(WHEEL_R + 0.2)
        .circle(WHEEL_R - RUBBER_GROOVE_D)
        .extrude(RUBBER_GROOVE_W)
    )
    rim = rim.cut(groove)
    # Solid hub
    hub = (
        cq.Workplane("XY")
        .workplane(offset=-1.0)
        .circle(7.0)
        .extrude(WHEEL_W + 2.0)
    )
    part = rim.union(hub)
    # D-shaft through hole
    bore = (
        _d_shaft_profile(cq.Workplane("XY"), clearance=0.12)
        .extrude(WHEEL_W + 4.0)
        .translate((0, 0, -2.0))
    )
    part = part.cut(bore)
    # A small set-screw boss (M2) on the hub, optional.
    boss_cut = (
        cq.Workplane("XZ")
        .workplane(offset=0)
        .center(0, WHEEL_W / 2.0)
        .circle(M2 / 2.0)
        .extrude(20)
        .translate((0, -10, 0))
    )
    # Push the M2 hole along +Y into the hub.
    set_screw = cq.Workplane("XZ").circle(M2 / 2.0).extrude(12).translate((0, 0, WHEEL_W / 2.0))
    # The XZ circle extrudes along +Y from y=0; shift so it crosses the axis.
    set_screw = (
        cq.Workplane("XZ")
        .workplane(offset=-12)
        .center(0, WHEEL_W / 2.0)
        .circle(M2 / 2.0)
        .extrude(24)
    )
    part = part.cut(set_screw)
    return part


def chassis_base() -> cq.Workplane:
    """U-channel chassis: floor, side walls, motor pockets, caster boss, standoffs."""
    floor_z = FLOOR_CLEARANCE
    # Origin: chassis centred in XY, floor bottom at z = 0 in the part
    # (we print it flat).  The real-world underside sits FLOOR_CLEARANCE
    # above the ground; that is a placement concern, not a solid concern.
    floor = cq.Workplane("XY").box(CHASSIS_L, CHASSIS_W, PLATE_T, centered=(True, True, False))

    # Side walls (left and right), running the full length.
    wall_t = 3.0
    wall = cq.Workplane("XY").box(CHASSIS_L, wall_t, SIDE_H, centered=(True, True, False))
    y_wall = (CHASSIS_W - wall_t) / 2.0
    walls = wall.translate((0, y_wall, 0)).union(wall.translate((0, -y_wall, 0)))
    part = floor.union(walls)

    # Rear wall (low, keeps the battery from sliding out).
    rear = (
        cq.Workplane("XY")
        .box(3.0, CHASSIS_W, 10.0, centered=(True, True, False))
        .translate((-(CHASSIS_L / 2.0) + 1.5, 0, 0))
    )
    part = part.union(rear)

    # Motor pockets: a rectangular tunnel through each side wall, shaft at
    # z = WHEEL_R - FLOOR_CLEARANCE = 14 mm above the floor underside.
    shaft_z = WHEEL_R - FLOOR_CLEARANCE  # 14 mm in the part frame
    motor_x = -CHASSIS_L / 2.0 + AXLE_X
    pocket = cq.Workplane("XY").box(N20_FACE_X, CHASSIS_W + 10, N20_FACE_Z, centered=True)
    pocket = pocket.translate((motor_x, 0, shaft_z))
    part = part.cut(pocket)

    # Clamp screw bosses beside each motor pocket (M3, two per side).
    for side in (1, -1):
        for dx in (-9.0, 9.0):
            boss = (
                cq.Workplane("XY")
                .box(6.0, 6.0, SIDE_H, centered=(True, True, False))
                .translate((motor_x + dx, side * (y_wall), 0))
            )
            part = part.union(boss)
            hole = (
                cq.Workplane("XY")
                .circle(M3 / 2.0)
                .extrude(SIDE_H + 2)
                .translate((motor_x + dx, side * y_wall, -1))
            )
            part = part.cut(hole)

    # Caster boss at the front, a 16 mm cylinder with an 8.6 mm bore and
    # a snap ring.  The cup is a separate part that press-fits here; the
    # boss is just a 12 mm hole + three M2 holes on a 16 mm circle in
    # case the student glues or screws the cup.
    caster_x = CHASSIS_L / 2.0 - 16.0
    boss = (
        cq.Workplane("XY")
        .circle(11.0)
        .extrude(PLATE_T + 2.0)
        .translate((caster_x, 0, 0))
    )
    part = part.union(boss)
    bore = (
        cq.Workplane("XY")
        .circle(8.2)
        .extrude(PLATE_T + 6.0)
        .translate((caster_x, 0, -1))
    )
    part = part.cut(bore)

    # Battery tray pocket: a 1 mm lip, open at the top, just ahead of the motors.
    batt_x = motor_x + N20_FACE_X / 2.0 + 6.0 + BATT_L / 2.0
    # Keep it on the floor, centred in Y.
    lip = (
        cq.Workplane("XY")
        .box(BATT_L + 4, BATT_W + 4, 6.0, centered=(True, True, False))
        .translate((batt_x, 0, PLATE_T))
    )
    well = (
        cq.Workplane("XY")
        .box(BATT_L, BATT_W, 8.0, centered=(True, True, False))
        .translate((batt_x, 0, PLATE_T))
    )
    part = part.union(lip).cut(well)

    # ESP32 standoffs on the front deck (four 6 mm posts, M2 holes,
    # 47.5 x 22 mm centres — typical DevKit V1 hole pattern is loose;
    # students can also zip-tie through the slots).
    esp_x = CHASSIS_L / 2.0 - 30.0
    for dx, dy in ((-23.5, 11.0), (23.5, 11.0), (-23.5, -11.0), (23.5, -11.0)):
        post = (
            cq.Workplane("XY")
            .circle(3.2)
            .extrude(8.0)
            .translate((esp_x + dx, dy, PLATE_T))
        )
        part = part.union(post)
        hole = (
            cq.Workplane("XY")
            .circle(M2 / 2.0)
            .extrude(12.0)
            .translate((esp_x + dx, dy, PLATE_T - 1))
        )
        part = part.cut(hole)

    # Zip-tie / cable slots near the driver and along the centreline.
    for x, y in ((esp_x, 0), (batt_x, CHASSIS_W / 2.0 - 14), (batt_x, -(CHASSIS_W / 2.0 - 14))):
        slot = (
            cq.Workplane("XY")
            .box(8.0, 2.4, PLATE_T + 4, centered=True)
            .translate((x, y, PLATE_T / 2.0))
        )
        part = part.cut(slot)

    # Lightening / access opening in the middle of the floor (optional, keeps
    # filament down and lets the student route wires).
    access = (
        cq.Workplane("XY")
        .box(24.0, 28.0, PLATE_T + 4, centered=True)
        .translate((0, 0, PLATE_T / 2.0))
        .edges("|Z")
        .fillet(3.0)
    )
    part = part.cut(access)

    # Front bumper lip, a place to clip the US bracket.
    bumper = (
        cq.Workplane("XY")
        .box(3.0, 40.0, 10.0, centered=(True, True, False))
        .translate((CHASSIS_L / 2.0 - 1.5, 0, PLATE_T))
    )
    part = part.union(bumper)
    for y in (-12.0, 12.0):
        h = (
            cq.Workplane("YZ")
            .workplane(offset=CHASSIS_L / 2.0 - 5)
            .center(y, PLATE_T + 5.0)
            .circle(M3 / 2.0)
            .extrude(10)
        )
        part = part.cut(h)

    return part


def motor_clamp() -> cq.Workplane:
    """A strap that closes the N20 pocket; two M3 holes match the chassis bosses."""
    w, d, t = 22.0, 8.0, 3.0
    part = cq.Workplane("XY").box(w, d, t, centered=(True, True, False))
    # Recess that hugs the motor body.
    recess = (
        cq.Workplane("XY")
        .box(N20_FACE_X, d + 1, 1.6, centered=(True, True, False))
        .translate((0, 0, 0))
    )
    part = part.cut(recess)
    for dx in (-9.0, 9.0):
        part = part.faces(">Z").workplane().center(dx, 0).hole(M3)
        # reset workplane by cutting a cylinder instead to avoid stacked centers
    part = cq.Workplane("XY").box(w, d, t, centered=(True, True, False))
    part = part.cut(
        cq.Workplane("XY").box(N20_FACE_X, d + 1, 1.6, centered=(True, True, False))
    )
    for dx in (-9.0, 9.0):
        hole = cq.Workplane("XY").circle(M3 / 2.0).extrude(t + 2).translate((dx, 0, -1))
        part = part.cut(hole)
    return part


def caster_cup() -> cq.Workplane:
    """Snap-in cup for an 8 mm steel bicycle ball (or bearing ball)."""
    ball_r = 4.0
    wall = 1.6
    outer_r = ball_r + wall + 0.2
    # Body: cylinder with a spherical cavity, opening slightly smaller than
    # the ball so it snaps in from below (print opening on the bed).
    body = cq.Workplane("XY").circle(outer_r).extrude(ball_r + wall + 2.5)
    # Flange to press into the chassis 16.4 mm hole (we use 16 mm boss / 8.2? 
    # The chassis has an 8.2 mm hole — too small.  Let's make the cup's
    # upper stem 8.0 mm to press into that hole, with a flange.)
    # Redo: cup sits under the plate; a 8.0 mm stem goes up through the hole
    # and a 1 mm flange rests against the underside.  Students can glue it.
    #
    # Simpler standalone cup: a 16 mm cylinder the student glues under the
    # front hole, cavity for the 8 mm ball, 7.2 mm retention opening.
    cavity = cq.CQ(cq.Solid.makeSphere(ball_r + 0.25)).translate((0, 0, ball_r + 0.8))
    part = body.cut(cavity)
    opening = (
        cq.Workplane("XY")
        .circle(ball_r - 0.4)
        .extrude(2.0)
    )
    part = part.cut(opening)
    # Side slot so the ball can be forced in and so we can print without
    # a trapped support volume — actually opening is on the bed, cavity
    # is above; overhang of the sphere is printable at 45°+ with a 0.2 mm
    # layer if we keep the opening on the bed.  Add a 1.5 mm slit anyway
    # so the student can pop the ball in after printing.
    slit = cq.Workplane("XY").box(1.6, outer_r * 2 + 2, 6.0, centered=(True, True, False))
    part = part.cut(slit)
    # Top stem
    stem = cq.Workplane("XY").workplane(offset=ball_r + wall + 2.5).circle(4.0).extrude(5.0)
    part = part.union(stem)
    return part


def sensor_bracket_us() -> cq.Workplane:
    """Front bracket for an HC-SR04: two 16 mm eyes and two M3 ears."""
    plate = cq.Workplane("XY").box(US_L + 8, US_W + 6, 3.0, centered=(True, True, False))
    # Sensor pocket
    pocket = (
        cq.Workplane("XY")
        .box(US_L, US_W, 2.0, centered=(True, True, False))
        .translate((0, 0, 1.5))
    )
    part = plate.cut(pocket)
    # Transducer holes
    for dx in (-US_HOLE_C / 2.0, US_HOLE_C / 2.0):
        h = cq.Workplane("XY").circle(US_HOLE_D / 2.0).extrude(5).translate((dx, 0, -1))
        part = part.cut(h)
    # Pin-header window
    window = cq.Workplane("XY").box(12.0, 8.0, 5.0, centered=True).translate((0, -US_W / 2.0 + 2, 1.5))
    part = part.cut(window)
    # Mounting ears with M3 holes, matching the chassis bumper (24 mm apart).
    for y in (-12.0, 12.0):
        ear = (
            cq.Workplane("XY")
            .box(8.0, 8.0, 3.0, centered=(True, True, False))
            .translate((-(US_L + 8) / 2.0 - 2.0, y, 0))
        )
        part = part.union(ear)
        hole = cq.Workplane("XY").circle(M3 / 2.0).extrude(5).translate((-(US_L + 8) / 2.0 - 2.0, y, -1))
        part = part.cut(hole)
    return part


def sensor_bracket_ir() -> cq.Workplane:
    """L-bracket for a 2–30 cm IR obstacle module (screws or zip-tie)."""
    back = cq.Workplane("XY").box(IR_W + 4, 3.0, 16.0, centered=(True, True, False))
    foot = (
        cq.Workplane("XY")
        .box(IR_W + 4, 16.0, 3.0, centered=(True, True, False))
        .translate((0, 8.0 - 1.5, 0))
    )
    part = back.union(foot)
    # Module pocket on the upright
    pocket = (
        cq.Workplane("XY")
        .box(IR_W, 1.6, 14.0, centered=(True, True, False))
        .translate((0, 0.7, 3.0))
    )
    part = part.cut(pocket)
    # Zip-tie slots in the foot and one M3
    for dx in (-6.0, 6.0):
        slot = cq.Workplane("XY").box(2.2, 6.0, 5.0, centered=True).translate((dx, 8.0, 1.5))
        part = part.cut(slot)
    hole = cq.Workplane("XY").circle(M3 / 2.0).extrude(5).translate((0, 10.0, -1))
    part = part.cut(hole)
    return part


def electronics_cover() -> cq.Workplane:
    """Optional dust cover over the ESP32 / driver area."""
    l, w, h, t = 70.0, 50.0, 18.0, 1.8
    outer = cq.Workplane("XY").box(l, w, h, centered=(True, True, False))
    inner = (
        cq.Workplane("XY")
        .box(l - 2 * t, w - 2 * t, h, centered=(True, True, False))
        .translate((0, 0, t))
    )
    part = outer.cut(inner)
    # USB / jumper windows at the front and one side
    usb = cq.Workplane("XY").box(12.0, 20.0, 10.0, centered=True).translate((l / 2.0, 0, 8.0))
    part = part.cut(usb)
    # Four 3.2 mm holes matching a subset of chassis posts (loosely, 47 x 22)
    for dx, dy in ((-23.5, 11.0), (23.5, 11.0), (-23.5, -11.0), (23.5, -11.0)):
        # holes through the top
        hole = (
            cq.Workplane("XY")
            .circle(M2 / 2.0 + 0.2)
            .extrude(t + 2)
            .translate((dx, dy, -1))
        )
        part = part.cut(hole)
    return part


def battery_retainer() -> cq.Workplane:
    """A simple strap that snaps over a 4xAA holder."""
    part = cq.Workplane("XY").box(BATT_L + 6, 8.0, 3.0, centered=(True, True, False))
    # Two downward hooks
    for dx in (-(BATT_L / 2.0 + 1.0), (BATT_L / 2.0 + 1.0)):
        hook = (
            cq.Workplane("XY")
            .box(3.0, 8.0, 14.0, centered=(True, True, False))
            .translate((dx, 0, 0))
        )
        part = part.union(hook)
        lip = (
            cq.Workplane("XY")
            .box(5.0, 8.0, 2.0, centered=(True, True, False))
            .translate((dx + (2.0 if dx < 0 else -2.0), 0, 0))
        )
        part = part.union(lip)
    return part


def wall_clip() -> cq.Workplane:
    """U-clip that holds 5 mm foam board (or 4 mm printed wall) upright."""
    gap = FOAM_T + 0.3
    jaw = 3.0
    depth = 12.0
    height = 18.0
    base = cq.Workplane("XY").box(CLIP_W, gap + 2 * jaw + 8.0, 3.0, centered=(True, True, False))
    left = (
        cq.Workplane("XY")
        .box(CLIP_W, jaw, height, centered=(True, True, False))
        .translate((0, (gap + jaw) / 2.0, 0))
    )
    right = (
        cq.Workplane("XY")
        .box(CLIP_W, jaw, height, centered=(True, True, False))
        .translate((0, -(gap + jaw) / 2.0, 0))
    )
    part = base.union(left).union(right)
    # Small ribs inside the jaws for grip
    for ysign in (1, -1):
        rib = (
            cq.Workplane("XY")
            .box(CLIP_W - 4, 0.6, height - 4, centered=(True, True, False))
            .translate((0, ysign * (gap / 2.0 - 0.1), 2))
        )
        part = part.union(rib)
    return part


def wall_post() -> cq.Workplane:
    """Corner / T-intersection post with four 5 mm slots, 50 mm tall."""
    w = POST_W
    h = WALL_H + 4.0
    gap = FOAM_T + 0.3
    body = cq.Workplane("XY").box(w, w, h, centered=(True, True, False))
    # Four slots from the top down, leaving a 4 mm floor.
    slot_h = WALL_H + 1.0
    for angle in (0, 90, 180, 270):
        slot = (
            cq.Workplane("XY")
            .box(w + 2, gap, slot_h, centered=(True, True, False))
            .translate((0, 0, 4.0))
            .rotate((0, 0, 0), (0, 0, 1), angle)
        )
        body = body.cut(slot)
    # Chamfer the top a little by a simple step (more printable than a chamfer
    # on a slotted solid).
    return body


def wall_segment(length: float = CELL - 4.0, handle: bool = False) -> cq.Workplane:
    """Solid PLA wall segment (optional; foam board is cheaper)."""
    part = cq.Workplane("XY").box(length, WALL_T, WALL_H, centered=(True, True, False))
    if handle:
        # A 12 mm tall flag/handle so the gate is obvious in the arena
        # and easy for the instructor to move mid-mission.
        handle_b = (
            cq.Workplane("XY")
            .box(16.0, WALL_T + 2.0, 14.0, centered=(True, True, False))
            .translate((0, 0, WALL_H))
        )
        flag = (
            cq.Workplane("XY")
            .box(22.0, 2.0, 16.0, centered=(True, True, False))
            .translate((8.0, 0, WALL_H + 8.0))
        )
        part = part.union(handle_b).union(flag)
    return part


def alignment_jig() -> cq.Workplane:
    """Go/no-go jig: motor shaft should sit at 24 mm above the table."""
    # A block 30 mm tall with a 3.2 mm horizontal hole at z=24, and a
    # 24 mm step so students can also just sight the shaft against the step.
    part = cq.Workplane("XY").box(30.0, 16.0, 30.0, centered=(True, True, False))
    hole = (
        cq.Workplane("XZ")
        .workplane(offset=-10)
        .center(0, 24.0)
        .circle(1.7)
        .extrude(20)
    )
    part = part.cut(hole)
    # Remove a front quarter above 24 mm so the shaft can be dropped in.
    cut = (
        cq.Workplane("XY")
        .box(16.0, 18.0, 8.0, centered=(True, True, False))
        .translate((7.0, 0, 24.0))
    )
    part = part.cut(cut)
    # Engraved-ish recess on the top (a shallow 24 numeral would be fancy;
    # a 2 mm deep arrow is enough).
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
    "wall_segment": lambda: wall_segment(handle=False),
    "gate_segment": lambda: wall_segment(handle=True),
    "alignment_jig": alignment_jig,
}


# Recommended print count per robot + one stock 5×5 arena.
PRINT_QTY = {
    "chassis_base": 1,
    "motor_clamp": 2,
    "wheel": 2,
    "caster_cup": 1,
    "sensor_bracket_us": 1,
    "sensor_bracket_ir": 2,
    "electronics_cover": 1,
    "battery_retainer": 1,
    "wall_clip": 24,
    "wall_post": 12,
    "wall_segment": 0,  # optional, foam board is the default
    "gate_segment": 1,
    "alignment_jig": 1,
}


def export_all(outdir: Path, stepdir: Path) -> list[dict]:
    outdir.mkdir(parents=True, exist_ok=True)
    stepdir.mkdir(parents=True, exist_ok=True)
    manifest = []
    for name, factory in PARTS.items():
        print(f"  building {name} ...", flush=True)
        solid = factory()
        stl = outdir / f"{name}.stl"
        step = stepdir / f"{name}.step"
        exporters.export(solid, str(stl))
        exporters.export(solid, str(step))
        manifest.append(
            {
                "name": name,
                "stl": str(stl.name),
                "step": str(step.name),
                "qty": PRINT_QTY[name],
            }
        )
    (outdir / "manifest.json").write_text(json.dumps(manifest, indent=2))
    return manifest


def main() -> int:
    here = Path(__file__).resolve().parent
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--outdir", type=Path, default=here.parent / "stl")
    p.add_argument("--stepdir", type=Path, default=here.parent / "step")
    args = p.parse_args()
    print("ShiftGate CadQuery export")
    export_all(args.outdir, args.stepdir)
    print("done")
    return 0


if __name__ == "__main__":
    sys.exit(main())
