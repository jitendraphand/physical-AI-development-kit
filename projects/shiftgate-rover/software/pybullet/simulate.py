#!/usr/bin/env python3
"""PyBullet simulation of the ShiftGate mission.

Runs the exact same `shiftgate_core.Mission` that the unit tests and the
ROS 2 node use.  The physics engine is used for rendering and for a
kinematic (reset-based) playback of the cell-to-cell path — the algorithm
is not hidden inside a one-off script.

Examples
--------
    # Just solve the task (no window, no video) — used by CI:
    python simulate.py --check

    # Headless simulation render (label this a *render*, not real footage):
    python simulate.py --record ../../media/shiftgate_sim.mp4

This file never pretends to be a camera recording of a physical robot.
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

# Make shiftgate_core importable without an install.
ROOT = Path(__file__).resolve().parents[1] / "shiftgate_core"
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from shiftgate_core.grid import Cell, Direction, DELTA  # noqa: E402
from shiftgate_core.maze import DemoMaze, demo_maze  # noqa: E402
from shiftgate_core.mission import (  # noqa: E402
    Mission,
    MissionConfig,
    MissionEvent,
    MissionPhase,
    sense_from_maze,
)
from shiftgate_core.occupancy import OccupancyMap  # noqa: E402
from shiftgate_core.params import (  # noqa: E402
    CELL_SIZE_M,
    CHASSIS_HEIGHT_M,
    CHASSIS_LENGTH_M,
    CHASSIS_WIDTH_M,
    GRID_N,
    WALL_HEIGHT_M,
    WALL_THICKNESS_M,
    WHEEL_RADIUS_M,
    WHEEL_SEPARATION_M,
)

# ---------------------------------------------------------------------------
# Mission trace
# ---------------------------------------------------------------------------


@dataclass
class TraceSample:
    phase: MissionPhase
    cell: Cell
    heading: Direction
    gate_closed: bool
    event_kinds: List[str] = field(default_factory=list)


def run_mission_trace(close_gate_on_return: bool = True, max_steps: int = 400) -> Tuple[List[TraceSample], List[MissionEvent], Mission]:
    maze = demo_maze()
    occ = OccupancyMap(n=maze.n)
    mission = Mission(occ, sense_from_maze(maze), MissionConfig(start=maze.start, package=maze.package))
    trace: List[TraceSample] = [
        TraceSample(mission.phase, mission.cell, mission.heading, maze.gate_closed, ["start"])
    ]
    closed = False
    for _ in range(max_steps):
        prev = mission.phase
        n_events = len(mission.events)
        phase = mission.step()
        if close_gate_on_return and not closed and prev != MissionPhase.RETURN and mission.phase == MissionPhase.RETURN:
            maze.close_gate()
            closed = True
        new_kinds = [e.kind for e in mission.events[n_events:]]
        trace.append(
            TraceSample(mission.phase, mission.cell, mission.heading, maze.gate_closed, new_kinds)
        )
        if phase in (MissionPhase.DONE, MissionPhase.FAILED):
            break
    return trace, mission.events, mission


# ---------------------------------------------------------------------------
# World + robot in PyBullet
# ---------------------------------------------------------------------------

# Visual palette (RGB 0-1)
COL_FLOOR = (0.86, 0.84, 0.78)
COL_START = (0.45, 0.72, 0.45)
COL_PKG = (0.95, 0.82, 0.30)
COL_WALL = (0.55, 0.58, 0.62)
COL_GATE = (0.95, 0.45, 0.12)
COL_CHASSIS = (0.12, 0.35, 0.48)
COL_COVER = (0.16, 0.55, 0.62)
COL_WHEEL = (0.12, 0.12, 0.12)
COL_GRID = (0.75, 0.73, 0.68)


def _cell_center(cell: Cell) -> Tuple[float, float]:
    x, y = cell
    # Arena origin at the outer south-west corner of cell (0,0), including wall.
    t = WALL_THICKNESS_M
    cx = t + CELL_SIZE_M * (x + 0.5)
    cy = t + CELL_SIZE_M * (y + 0.5)
    return cx, cy


def _arena_outer() -> float:
    return GRID_N * CELL_SIZE_M + (GRID_N + 1) * WALL_THICKNESS_M


def _heading_yaw(h: Direction) -> float:
    # Body +x is forward.  EAST = 0, NORTH = pi/2, WEST = pi, SOUTH = -pi/2
    return {Direction.EAST: 0.0, Direction.NORTH: math.pi / 2, Direction.WEST: math.pi, Direction.SOUTH: -math.pi / 2}[h]


def _rgba(rgb, a=1.0):
    return list(rgb) + [a]


class ShiftGateWorld:
    def __init__(self, pb, maze: DemoMaze):
        self.pb = pb
        self.maze = maze
        self.gate_body = None
        self.robot = None
        self.wheel_l = None
        self.wheel_r = None
        self._build()

    def _box(self, half, pos, rgb, mass=0.0, z_up=True):
        col = self.pb.createCollisionShape(self.pb.GEOM_BOX, halfExtents=half)
        vis = self.pb.createVisualShape(self.pb.GEOM_BOX, halfExtents=half, rgbaColor=_rgba(rgb))
        return self.pb.createMultiBody(mass, col, vis, pos)

    def _build(self):
        pb = self.pb
        pb.resetSimulation()
        pb.setGravity(0, 0, -9.81)
        outer = _arena_outer()
        # Floor slightly larger than the arena
        margin = 0.08
        self._box(
            [outer / 2 + margin, outer / 2 + margin, 0.008],
            [outer / 2, outer / 2, -0.008],
            COL_FLOOR,
        )
        # Cell tiles (start / package / faint grid)
        for x in range(GRID_N):
            for y in range(GRID_N):
                cx, cy = _cell_center((x, y))
                if (x, y) == self.maze.start:
                    rgb = COL_START
                elif (x, y) == self.maze.package:
                    rgb = COL_PKG
                else:
                    rgb = COL_GRID if (x + y) % 2 == 0 else tuple(c * 0.96 for c in COL_GRID)
                half = CELL_SIZE_M / 2 - 0.004
                self._box([half, half, 0.001], [cx, cy, 0.001], rgb)

        # Walls: iterate all 4-adjacent edges plus the outer boundary.
        t = WALL_THICKNESS_M
        h = WALL_HEIGHT_M
        n = GRID_N

        def add_hwall(x, y_south, rgb=COL_WALL, is_gate=False):
            # Horizontal wall (east-west) on the south side of cells in row y_south,
            # or the north outer when y_south == n.
            cx = t + CELL_SIZE_M * (x + 0.5)
            cy = t + CELL_SIZE_M * y_south + (0 if y_south == 0 or y_south == n else 0)
            # The wall centre sits on the grid line between y-1 and y (or outer).
            cy = WALL_THICKNESS_M / 2 + y_south * CELL_SIZE_M + y_south * 0  # wait
            # Grid line y_index runs along y = y_index * CELL + (y_index+0.5)*t? 
            # Outer south line at y = t/2, then each next line is + CELL + t.
            cy = t / 2 + y_south * (CELL_SIZE_M + t)
            cx = t + CELL_SIZE_M * (x + 0.5) + 0  # cell centres already include one t offset
            # Cell (x,0) centre y = t + CELL/2.  South outer wall centre y = t/2. Yes.
            cx = t + CELL_SIZE_M * (x + 0.5)
            z = h / 2
            half = [CELL_SIZE_M / 2 + t / 2, t / 2, h / 2]
            body = self._box(half, [cx, cy, z], rgb)
            if is_gate:
                self.gate_body = body
                self._gate_open_pos = [cx, cy, z]
                self._gate_closed_pos = [cx, cy, z]  # same xy; we "drop in" from above
                self._gate_hidden_pos = [cx, cy, z + 2.0]  # well above the camera while the gate is open

        def add_vwall(x_west, y, rgb=COL_WALL, is_gate=False):
            cx = t / 2 + x_west * (CELL_SIZE_M + t)
            cy = t + CELL_SIZE_M * (y + 0.5)
            z = h / 2
            half = [t / 2, CELL_SIZE_M / 2 + t / 2, h / 2]
            body = self._box(half, [cx, cy, z], rgb)
            if is_gate:
                self.gate_body = body
                self._gate_open_pos = [cx, cy, z]
                self._gate_hidden_pos = [cx, cy, z + 2.0]  # well above the camera while the gate is open

        # Outer walls
        for x in range(n):
            add_hwall(x, 0)
            add_hwall(x, n)
        for y in range(n):
            add_vwall(0, y)
            add_vwall(n, y)

        # Internal walls from the maze.  The gate starts OPEN so it is not
        # in maze.blocked; we still spawn its body in the *hidden* (raised)
        # pose and drop it when the gate closes.
        gx, gy = self.maze.gate
        # gate is a north-south door (same x, y differs) in the stock maze.
        for (a, b) in self.maze.blocked:
            if a[0] == b[0] and abs(a[1] - b[1]) == 1:
                x = a[0]
                y_line = max(a[1], b[1])
                add_hwall(x, y_line)
            elif a[1] == b[1] and abs(a[0] - b[0]) == 1:
                y = a[1]
                x_line = max(a[0], b[0])
                add_vwall(x_line, y)

        # Gate body (stock maze: horizontal door between (2,2) and (2,3))
        ga, gb = self.maze.gate
        if ga[0] == gb[0]:
            add_hwall(ga[0], max(ga[1], gb[1]), rgb=COL_GATE, is_gate=True)
        else:
            add_vwall(max(ga[0], gb[0]), ga[1], rgb=COL_GATE, is_gate=True)
        # Start hidden (gate is open).
        if self.gate_body is not None:
            pb.resetBasePositionAndOrientation(
                self.gate_body, self._gate_hidden_pos, [0, 0, 0, 1]
            )
            self._gate_is_closed = False

        self._build_robot()

    def set_gate_closed(self, closed: bool, frac: float = 1.0):
        """frac=0 hidden (open), frac=1 in place (closed)."""
        if self.gate_body is None:
            return
        ho = self._gate_hidden_pos
        op = self._gate_open_pos
        pos = [ho[i] + (op[i] - ho[i]) * frac for i in range(3)]
        self.pb.resetBasePositionAndOrientation(self.gate_body, pos, [0, 0, 0, 1])
        self._gate_is_closed = closed and frac >= 0.99

    def _build_robot(self):
        pb = self.pb
        # Simple but readable: a box chassis, two cylinders as wheels, a
        # small caster sphere, and a "sensor bar" in front.  Optionally we
        # could load the printed STLs; boxes composite more reliably in
        # TinyRenderer and keep the file self-contained.
        l, w, h = CHASSIS_LENGTH_M, CHASSIS_WIDTH_M, CHASSIS_HEIGHT_M
        z_floor = 0.010  # floor underside
        chassis_z = z_floor + h / 2
        col = pb.createCollisionShape(pb.GEOM_BOX, halfExtents=[l / 2, w / 2, h / 2])
        vis = pb.createVisualShape(pb.GEOM_BOX, halfExtents=[l / 2, w / 2, h / 2], rgbaColor=_rgba(COL_CHASSIS))
        self.robot = pb.createMultiBody(0.4, col, vis, [0, 0, chassis_z])
        # Cover
        ch = 0.014
        col_c = pb.createCollisionShape(pb.GEOM_BOX, halfExtents=[0.034, 0.024, ch / 2])
        vis_c = pb.createVisualShape(pb.GEOM_BOX, halfExtents=[0.034, 0.024, ch / 2], rgbaColor=_rgba(COL_COVER))
        self.cover = pb.createMultiBody(0.02, col_c, vis_c, [0.02, 0, chassis_z + h / 2 + ch / 2])
        # Wheels
        wr = WHEEL_RADIUS_M
        ww = 0.008
        col_w = pb.createCollisionShape(pb.GEOM_CYLINDER, radius=wr, height=ww)
        vis_w = pb.createVisualShape(pb.GEOM_CYLINDER, radius=wr, length=ww, rgbaColor=_rgba(COL_WHEEL))
        self.wheel_l = pb.createMultiBody(0.03, col_w, vis_w, [0, 0, wr])
        self.wheel_r = pb.createMultiBody(0.03, col_w, vis_w, [0, 0, wr])
        # Front ultrasonic "eyes"
        eye_r = 0.008
        col_e = pb.createCollisionShape(pb.GEOM_CYLINDER, radius=eye_r, height=0.012)
        vis_e = pb.createVisualShape(pb.GEOM_CYLINDER, radius=eye_r, length=0.012, rgbaColor=[0.9, 0.9, 0.85, 1])
        self.eye_l = pb.createMultiBody(0.0, col_e, vis_e, [0, 0, 0])
        self.eye_r = pb.createMultiBody(0.0, col_e, vis_e, [0, 0, 0])
        self._wheel_angle = 0.0
        self.place((0, 0), Direction.EAST, 0.0)

    def place(self, cell: Cell, heading: Direction, wheel_angle: float = 0.0, offset: Optional[Tuple[float, float]] = None):
        cx, cy = _cell_center(cell)
        if offset:
            cx, cy = offset
        yaw = _heading_yaw(heading)
        q = self.pb.getQuaternionFromEuler([0, 0, yaw])
        l, w, h = CHASSIS_LENGTH_M, CHASSIS_WIDTH_M, CHASSIS_HEIGHT_M
        z_floor = 0.010
        chassis_z = z_floor + h / 2
        self.pb.resetBasePositionAndOrientation(self.robot, [cx, cy, chassis_z], q)
        # Cover sits slightly forward on the deck
        fx, fy = math.cos(yaw), math.sin(yaw)
        lx, ly = -fy, fx  # left
        self.pb.resetBasePositionAndOrientation(
            self.cover, [cx + 0.020 * fx, cy + 0.020 * fy, chassis_z + h / 2 + 0.007], q
        )
        # Wheels: cylinder default axis is +Z in PyBullet, so we rotate them
        # to the y-axis (axle) and then apply yaw.  Roll around the axle for
        # a spinning effect.
        wr = WHEEL_RADIUS_M
        half = WHEEL_SEPARATION_M / 2
        # Wheel visual: rotate so cylinder axis is along body +y, plus roll.
        q_wheel_base = self.pb.getQuaternionFromEuler([math.pi / 2, 0, yaw])
        # Additional roll around axle: compose a rotation about body +y.
        q_roll = self.pb.getQuaternionFromEuler([math.pi / 2, wheel_angle, yaw])
        for body, sign in ((self.wheel_l, 1.0), (self.wheel_r, -1.0)):
            px = cx + sign * half * lx
            py = cy + sign * half * ly
            self.pb.resetBasePositionAndOrientation(body, [px, py, wr], q_roll)
        # Eyes at the front face
        q_eye = self.pb.getQuaternionFromEuler([0, math.pi / 2, yaw])
        for body, sign in ((self.eye_l, 0.012), (self.eye_r, -0.012)):
            px = cx + (l / 2 + 0.004) * fx + sign * lx
            py = cy + (l / 2 + 0.004) * fy + sign * ly
            self.pb.resetBasePositionAndOrientation(body, [px, py, chassis_z + 0.004], q_eye)
        self._pose = (cx, cy, yaw)


# ---------------------------------------------------------------------------
# Camera + HUD
# ---------------------------------------------------------------------------


def camera_image(pb, eye, target, width, height, up=(0, 0, 1), fov=55.0):
    view = pb.computeViewMatrix(eye, target, up)
    aspect = width / height
    proj = pb.computeProjectionMatrixFOV(fov, aspect, 0.04, 3.5)
    _, _, rgb, _, _ = pb.getCameraImage(
        width,
        height,
        view,
        proj,
        renderer=pb.ER_TINY_RENDERER,
        flags=pb.ER_NO_SEGMENTATION_MASK,
    )
    rgb = np.reshape(rgb, (height, width, 4))[:, :, :3].astype(np.uint8)
    return rgb


def overlay(rgb: np.ndarray, lines: List[str], banner: Optional[str] = None, footer: Optional[str] = None) -> np.ndarray:
    """Draw a simple HUD with Pillow.  Falls back to raw frames if Pillow is missing."""
    try:
        from PIL import Image, ImageDraw, ImageFont
    except ImportError:
        return rgb
    img = Image.fromarray(rgb)
    draw = ImageDraw.Draw(img, "RGBA")
    # Try a clean sans; TinyRenderer has no text of its own.
    try:
        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 18)
        font_b = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 22)
        font_s = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 14)
        font_t = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 28)
    except Exception:
        font = font_b = font_s = font_t = ImageFont.load_default()
    w, h = img.size
    # Top bar
    draw.rectangle([0, 0, w, 36], fill=(12, 18, 24, 200))
    draw.text((12, 8), "ShiftGate Rover  ·  simulation render (not real footage)", font=font_s, fill=(230, 230, 230, 255))
    y = 48
    for i, line in enumerate(lines):
        # Chip background
        tw = 12 + (len(line) * 10)
        draw.rounded_rectangle([8, y, min(w - 8, 8 + tw), y + 26], radius=4, fill=(12, 18, 24, 180))
        draw.text((16, y + 3), line, font=font, fill=(255, 255, 255, 255))
        y += 30
    if banner:
        bw, bh = w, 64
        draw.rectangle([0, h // 2 - bh // 2, w, h // 2 + bh // 2], fill=(180, 60, 20, 200))
        # centre the banner
        try:
            bbox = draw.textbbox((0, 0), banner, font=font_t)
            tw, th = bbox[2] - bbox[0], bbox[3] - bbox[1]
        except Exception:
            tw, th = 8 * len(banner), 24
        draw.text(((w - tw) / 2, (h - th) / 2 - 2), banner, font=font_t, fill=(255, 255, 255, 255))
    if footer:
        draw.rectangle([0, h - 28, w, h], fill=(12, 18, 24, 200))
        draw.text((12, h - 22), footer, font=font_s, fill=(210, 210, 210, 255))
    return np.asarray(img)


# ---------------------------------------------------------------------------
# Playback
# ---------------------------------------------------------------------------


PHASE_LABEL = {
    MissionPhase.EXPLORE: "Phase 1  ·  Explore & map an unknown maze",
    MissionPhase.FETCH: "Phase 2  ·  Shortest path to the package",
    MissionPhase.RETURN: "Phase 3  ·  Shortest path home",
    MissionPhase.REEXPLORE: "Re-explore  ·  searching for a new route",
    MissionPhase.DONE: "Home  ·  mission complete",
    MissionPhase.FAILED: "Mission failed",
}


def lerp(a, b, t):
    return a + (b - a) * t


def slerp_angle(a, b, t):
    # shortest arc
    d = (b - a + math.pi) % (2 * math.pi) - math.pi
    return a + d * t


def play_and_record(
    pb,
    world: ShiftGateWorld,
    trace: List[TraceSample],
    events: List[MissionEvent],
    out: Optional[Path],
    width: int,
    height: int,
    fps: int,
    title_s: float = 2.2,
) -> Optional[Path]:
    frames: List[np.ndarray] = []
    outer = _arena_outer()
    centre = (outer / 2, outer / 2, 0.0)

    def cam_for(cx, cy, mode="follow"):
        if mode == "top":
            eye = [centre[0], centre[1] - 0.05, 1.55]
            target = [centre[0], centre[1], 0.0]
        elif mode == "oblique":
            eye = [centre[0] - 0.55, centre[1] - 0.85, 0.85]
            target = [centre[0] + 0.05, centre[1] + 0.05, 0.0]
        else:
            # follow, slightly behind and above the robot
            eye = [cx - 0.38, cy - 0.42, 0.42]
            target = [cx + 0.05, cy + 0.05, 0.02]
        return eye, target

    def push(rgb, lines, banner=None, footer="simulation render — PyBullet TinyRenderer"):
        frames.append(overlay(rgb, lines, banner=banner, footer=footer))

    # Title cards
    n_title = int(title_s * fps)
    world.place(trace[0].cell, trace[0].heading, 0.0)
    world.set_gate_closed(False, 0.0)
    eye, target = cam_for(*_cell_center(trace[0].cell), mode="oblique")
    for i in range(n_title):
        rgb = camera_image(pb, eye, target, width, height)
        banner = "ShiftGate Rover" if i < n_title * 0.7 else None
        extra = "Map  ·  Detect a silent gate shift  ·  Replan home" if i < n_title * 0.7 else None
        lines = [extra] if extra else []
        push(rgb, lines, banner=banner)

    # Build a waypoint list: each trace sample is a cell.  When the cell or
    # heading changes, we animate a turn and/or a straight drive.
    wheel = 0.0
    gate_frac = 0.0
    current_phase = trace[0].phase
    # Group samples
    pose_cell = trace[0].cell
    pose_head = trace[0].heading

    def render_pose(cell, heading, offset, wheel, phase, status, mode, banner=None, gate=None):
        world.place(cell, heading, wheel, offset=offset)
        if gate is not None:
            world.set_gate_closed(gate >= 0.99, gate)
        cx, cy = offset
        eye, target = cam_for(cx, cy, mode=mode)
        rgb = camera_image(pb, eye, target, width, height)
        lines = [PHASE_LABEL.get(phase, phase.value), status]
        push(rgb, [ln for ln in lines if ln], banner=banner)

    def animate_move(c0, h0, c1, h1, phase, status, seconds, mode="follow", banner=None):
        nonlocal wheel, gate_frac
        x0, y0 = _cell_center(c0)
        x1, y1 = _cell_center(c1)
        yaw0, yaw1 = _heading_yaw(h0), _heading_yaw(h1)
        n = max(1, int(seconds * fps))
        # If we must turn, spend the first 35% turning in place.
        need_turn = abs((yaw1 - yaw0 + math.pi) % (2 * math.pi) - math.pi) > 0.05
        n_turn = int(n * 0.35) if need_turn and (c0 == c1 or True) else 0
        if c0 == c1:
            n_turn = n  # pure turn / dwell
        for i in range(n):
            t = (i + 1) / n
            if n_turn and i < n_turn:
                tt = (i + 1) / max(n_turn, 1)
                yaw = slerp_angle(yaw0, yaw1, tt)
                # fake a heading enum by passing h1 only at the end; we pass
                # offset and a temporary heading.  place() uses heading for
                # yaw, so we temporarily... we'll pass offset and heading
                # interpolated via a custom yaw by faking Direction? Easier:
                # extend place to accept yaw.  For now, set heading to h1
                # only after the turn; during the turn, interpolate by
                # calling a small helper.
                offset = (x0, y0)
                # Use a synthetic place with custom yaw:
                _place_yaw(world, x0, y0, yaw, wheel)
            else:
                tt = (i + 1 - n_turn) / max(n - n_turn, 1)
                offset = (lerp(x0, x1, tt), lerp(y0, y1, tt))
                yaw = yaw1
                dist = math.hypot(x1 - x0, y1 - y0)
                wheel += (dist / max(n - n_turn, 1)) / WHEEL_RADIUS_M
                _place_yaw(world, offset[0], offset[1], yaw, wheel)
            world.set_gate_closed(gate_frac >= 0.99, gate_frac)
            eye, target = cam_for(offset[0] if i >= n_turn else x0, offset[1] if i >= n_turn else y0, mode=mode)
            rgb = camera_image(pb, eye, target, width, height)
            lines = [PHASE_LABEL.get(phase, phase.value)]
            if status:
                lines.append(status)
            push(rgb, lines, banner=banner)

    # Walk the trace
    status = "Building a witnessed occupancy map"
    cam_mode = "oblique"
    banner_hold = 0
    for i in range(1, len(trace)):
        prev, cur = trace[i - 1], trace[i]
        # Status / banners from events
        banner = None
        if "explored" in cur.event_kinds:
            status = "Map complete  ·  every reachable cell visited"
        if "fetch_plan" in cur.event_kinds:
            status = "A* planned a path to the package"
        if "fetched" in cur.event_kinds:
            status = "Package reached  ·  planning the shortest path home"
        if "discrepancy" in cur.event_kinds:
            status = "Discrepancy  ·  BLOCKED_FREE  ·  a believed-open edge is now a wall"
            banner = "GATE SHIFT DETECTED"
        if "replan" in cur.event_kinds and "discrepancy" not in cur.event_kinds and cur.phase == MissionPhase.RETURN:
            if any("new home path" in (e.detail if hasattr(e, "detail") else "") for e in events):
                pass
            if prev.phase != MissionPhase.RETURN:
                status = "Shortest path home  ·  8 cells, through the centre shaft"
            else:
                status = "Replanned  ·  the short shaft is closed; taking the 12-cell detour"
        if "home" in cur.event_kinds:
            status = "Returned to start  ·  mission complete"
            banner = "HOME"
        if cur.phase == MissionPhase.EXPLORE:
            status = status if "explored" not in cur.event_kinds else status
            if "explored" not in cur.event_kinds:
                status = f"Exploring  ·  visiting cell {cur.cell}"
            cam_mode = "follow" if i > 3 else "oblique"
            seconds = 0.28
        elif cur.phase == MissionPhase.FETCH:
            cam_mode = "follow"
            seconds = 0.45
        elif cur.phase == MissionPhase.RETURN:
            cam_mode = "follow"
            seconds = 0.55
        else:
            seconds = 0.35
            cam_mode = "follow"

        # Animate the gate drop when it first closes.
        if cur.gate_closed and not prev.gate_closed:
            # Drop the gate over 0.8 s with a banner, robot idle.
            n_drop = int(0.9 * fps)
            x, y = _cell_center(prev.cell)
            yaw = _heading_yaw(prev.heading)
            for k in range(n_drop):
                gate_frac = (k + 1) / n_drop
                world.set_gate_closed(False, gate_frac)
                _place_yaw(world, x, y, yaw, wheel)
                eye, target = cam_for(x, y, mode="oblique")
                rgb = camera_image(pb, eye, target, width, height)
                push(
                    rgb,
                    [PHASE_LABEL[MissionPhase.RETURN], "The instructor slides the gate shut  ·  the robot is not told"],
                    banner="SILENT GATE SHIFT",
                )
            gate_frac = 1.0
            world.set_gate_closed(True, 1.0)

        # Dwell a moment on a banner
        if prev.cell == cur.cell and prev.heading == cur.heading:
            # still animate a short dwell so events are visible
            animate_move(prev.cell, prev.heading, cur.cell, cur.heading, cur.phase, status, 0.18 if not banner else 0.7, mode=cam_mode, banner=banner)
        else:
            animate_move(prev.cell, prev.heading, cur.cell, cur.heading, cur.phase, status, seconds, mode=cam_mode, banner=banner)

    # End hold, top-down
    if trace:
        last = trace[-1]
        x, y = _cell_center(last.cell)
        yaw = _heading_yaw(last.heading)
        _place_yaw(world, x, y, yaw, wheel)
        world.set_gate_closed(last.gate_closed, 1.0 if last.gate_closed else 0.0)
        for k in range(int(2.0 * fps)):
            # Ease from follow to top
            t = min(1.0, k / (0.6 * fps))
            eye_f, tgt_f = cam_for(x, y, "follow")
            eye_t, tgt_t = cam_for(x, y, "top")
            eye = [lerp(eye_f[i], eye_t[i], t) for i in range(3)]
            target = [lerp(tgt_f[i], tgt_t[i], t) for i in range(3)]
            rgb = camera_image(pb, eye, target, width, height)
            push(
                rgb,
                [PHASE_LABEL.get(last.phase, last.phase.value), "Returned home after a discrepancy-triggered replan"],
                banner="HOME" if k < 0.7 * fps else None,
            )

    if not out:
        return None
    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    _write_mp4(frames, out, fps)
    return out


def _place_yaw(world: ShiftGateWorld, x, y, yaw, wheel):
    # Reconstruct a Direction-less placement using the existing helper
    # by temporarily faking the closest heading and then overwriting the
    # orientation.  Cleaner: just set bodies here.
    pb = world.pb
    l, w, h = CHASSIS_LENGTH_M, CHASSIS_WIDTH_M, CHASSIS_HEIGHT_M
    z_floor = 0.010
    chassis_z = z_floor + h / 2
    q = pb.getQuaternionFromEuler([0, 0, yaw])
    pb.resetBasePositionAndOrientation(world.robot, [x, y, chassis_z], q)
    fx, fy = math.cos(yaw), math.sin(yaw)
    lx, ly = -fy, fx
    pb.resetBasePositionAndOrientation(
        world.cover, [x + 0.020 * fx, y + 0.020 * fy, chassis_z + h / 2 + 0.007], q
    )
    wr = WHEEL_RADIUS_M
    half = WHEEL_SEPARATION_M / 2
    q_roll = pb.getQuaternionFromEuler([math.pi / 2, wheel, yaw])
    for body, sign in ((world.wheel_l, 1.0), (world.wheel_r, -1.0)):
        px = x + sign * half * lx
        py = y + sign * half * ly
        pb.resetBasePositionAndOrientation(body, [px, py, wr], q_roll)
    q_eye = pb.getQuaternionFromEuler([0, math.pi / 2, yaw])
    for body, sign in ((world.eye_l, 0.012), (world.eye_r, -0.012)):
        px = x + (l / 2 + 0.004) * fx + sign * lx
        py = y + (l / 2 + 0.004) * fy + sign * ly
        pb.resetBasePositionAndOrientation(body, [px, py, chassis_z + 0.004], q_eye)


def _write_mp4(frames: List[np.ndarray], out: Path, fps: int) -> None:
    import subprocess
    import tempfile
    import shutil

    tmp = Path(tempfile.mkdtemp(prefix="shiftgate_frames_"))
    try:
        from PIL import Image
        for i, fr in enumerate(frames):
            Image.fromarray(fr).save(tmp / f"f_{i:05d}.png")
        cmd = [
            "ffmpeg", "-y", "-framerate", str(fps),
            "-i", str(tmp / "f_%05d.png"),
            "-c:v", "libx264", "-pix_fmt", "yuv420p",
            "-crf", "23", "-movflags", "+faststart",
            "-vf", "format=yuv420p",
            str(out),
        ]
        subprocess.check_call(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--check", action="store_true", help="run the mission with no rendering")
    p.add_argument("--record", type=Path, default=None, help="write a simulation-render MP4")
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--fps", type=int, default=20)
    p.add_argument("--seed", type=int, default=0)
    args = p.parse_args()

    trace, events, mission = run_mission_trace()
    kinds = [e.kind for e in events]
    ok = mission.phase == MissionPhase.DONE and "discrepancy" in kinds and "replan" in kinds
    print(f"mission phase={mission.phase.value} cell={mission.cell} steps={len(trace)}")
    print("events:", kinds)
    if not ok:
        print("FAIL: expected DONE with a discrepancy + replan", file=sys.stderr)
        if args.check and not args.record:
            return 1
    else:
        print("CHECK OK: explored, detected a silent gate shift, replanned, got home")
    if args.check and not args.record:
        return 0 if ok else 1

    import pybullet as pb
    import pybullet_data

    cid = pb.connect(pb.DIRECT)
    pb.setAdditionalSearchPath(pybullet_data.getDataPath())
    world = ShiftGateWorld(pb, demo_maze())
    out = args.record
    if out is None and not args.check:
        out = Path(__file__).resolve().parents[2] / "media" / "shiftgate_sim.mp4"
    if out:
        print(f"recording {len(trace)} trace samples → {out}")
        play_and_record(pb, world, trace, events, out, args.width, args.height, args.fps)
        size = out.stat().st_size if out.exists() else 0
        print(f"wrote {out} ({size/1e6:.1f} MB)")
    pb.disconnect()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
