"""Stock curriculum maze: a zigzag of corridors plus one critical gate.

The 5×5 arena is five open east–west corridors stacked north–south.  Adjacent
corridors connect through staggered "doors" at the ends, which forces a long
snake walk from the far corner back to start.  A *shortcut shaft* down the
centre column (x = 2) cuts that walk from 24 steps to 8 — but only while the
**gate** (the door between (2, 2) and (2, 3)) stays open.

When the instructor (or the simulator) closes the gate, every 8-step path
disappears and the shortest remaining path is 16 steps.  That is the
discrepancy the rover has to detect and replan around.

The gate is an ordinary 3D-printed wall tile with a handle; nothing on the
robot is told that it moved.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable, Optional, Set, Tuple

from .grid import Cell, Direction, Edge, GridGeom, neighbour, undirected
from .params import GRID_N, PACKAGE_CELL, START_CELL

# Horizontal door between (x, y) and (x, y+1), stored as (x, y).
HorizDoor = Tuple[int, int]


def _zigzag_blocked_horiz(n: int = GRID_N) -> Set[HorizDoor]:
    """Doors that are walled in the stock zigzag (before the shaft/gate)."""
    blocked: Set[HorizDoor] = set()
    for y in range(n - 1):
        if y % 2 == 0:
            # Door only at the east end (x = n-1).
            open_x = n - 1
        else:
            # Door only at the west end (x = 0).
            open_x = 0
        for x in range(n):
            if x != open_x:
                blocked.add((x, y))
    return blocked


@dataclass
class DemoMaze:
    """Ground-truth arena.  Unknown to the robot; used by sims and tests."""

    n: int = GRID_N
    start: Cell = START_CELL
    package: Cell = PACKAGE_CELL
    # Blocked 4-adjacent edges (undirected).  Outer boundary is implicit.
    blocked: Set[Edge] = field(default_factory=set)
    # The one edge the instructor can flip mid-mission.
    gate: Edge = ((2, 2), (2, 3))
    gate_closed: bool = False

    def __post_init__(self) -> None:
        self.geom = GridGeom(self.n)
        self.gate = undirected(*self.gate)
        self.blocked = {undirected(*e) for e in self.blocked}

    def is_blocked(self, a: Cell, b: Cell) -> bool:
        e = undirected(a, b)
        if e == self.gate:
            return self.gate_closed
        return e in self.blocked

    def close_gate(self) -> None:
        self.gate_closed = True

    def open_gate(self) -> None:
        self.gate_closed = False

    def neighbours(self, cell: Cell) -> Iterable[Cell]:
        for _, nxt in self.geom.neighbours(cell):
            if not self.is_blocked(cell, nxt):
                yield nxt

    def sense_range(self, cell: Cell, heading: Direction, max_cells: int = 2) -> Optional[float]:
        """Return distance in *cells* to the first wall along heading, or None if
        the view exits the useful range without a hit.

        A wall immediately in front of `cell` (the edge to the next cell, or
        the outer boundary) returns 0.5 — the distance from cell centre to
        the face.  A wall one cell further returns 1.5, and so on.
        """
        cur = cell
        for step in range(max_cells):
            nxt = neighbour(cur, heading)
            if not self.geom.in_bounds(nxt) or self.is_blocked(cur, nxt):
                return step + 0.5
            cur = nxt
        return None


def demo_maze() -> DemoMaze:
    """Build the stock curriculum maze (gate open)."""
    n = GRID_N
    blocked: Set[Edge] = set()
    zigzag = _zigzag_blocked_horiz(n)
    # Shortcut shaft: permanently open the centre-column doors except the gate.
    shaft_x = 2
    gate_door: HorizDoor = (2, 2)  # between (2,2) and (2,3)
    for x, y in zigzag:
        if x == shaft_x and (x, y) != gate_door:
            continue  # leave the shaft open
        a = (x, y)
        b = (x, y + 1)
        blocked.add(undirected(a, b))
    # The gate starts OPEN, so it is *not* in `blocked`.
    return DemoMaze(
        n=n,
        start=START_CELL,
        package=PACKAGE_CELL,
        blocked=blocked,
        gate=undirected((2, 2), (2, 3)),
        gate_closed=False,
    )


def ascii_maze(maze: DemoMaze, robot: Optional[Cell] = None) -> str:
    """Render a text map.  `=` marks the gate (open or closed)."""
    n = maze.n
    lines = []
    # Top outer wall.
    lines.append("  " + "+---" * n + "+")
    for y in range(n - 1, -1, -1):
        row = f"{y} |"
        for x in range(n):
            cell = (x, y)
            if robot == cell:
                glyph = " R "
            elif maze.start == cell:
                glyph = " S "
            elif maze.package == cell:
                glyph = " P "
            else:
                glyph = "   "
            row += glyph
            east = (x + 1, y)
            if x == n - 1:
                row += "|"
            else:
                edge = undirected(cell, east)
                row += "|" if maze.is_blocked(cell, east) else " "
                _ = edge
        lines.append(row)
        # Horizontal walls below this row (south, toward smaller y).
        if y == 0:
            lines.append("  " + "+---" * n + "+")
        else:
            seg = "  +"
            for x in range(n):
                south = (x, y - 1)
                edge = undirected((x, y), south)
                if edge == maze.gate:
                    seg += "===" if maze.gate_closed else "   "
                elif maze.is_blocked((x, y), south):
                    seg += "---"
                else:
                    seg += "   "
                seg += "+"
            lines.append(seg)
    # x labels
    xlabel = "    " + "   ".join(str(x) for x in range(n))
    lines.append(xlabel)
    return "\n".join(lines)
