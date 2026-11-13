"""Witnessed occupancy grid: cells and edges with decaying trust."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, Iterable, Optional, Set

from .grid import Cell, Direction, Edge, GridGeom, neighbour, undirected
from .params import GRID_N, TRUST_HALF_LIFE_STEPS


class CellState(Enum):
    UNKNOWN = "unknown"
    FREE = "free"
    # A cell is never marked OCCUPIED in this project: obstacles are walls
    # on *edges*.  The enum is reserved so students can extend the model
    # to occupied cells (e.g. a moved box in the challenge extension).
    OCCUPIED = "occupied"


class EdgeState(Enum):
    UNKNOWN = "unknown"
    OPEN = "open"
    WALL = "wall"
    # An opening that looks like a one-cell gap in a longer wall — the
    # kind of feature an instructor can flip.  Treated as OPEN for
    # planning, but highlighted for the student in RViz / the sim HUD.
    GATE_CANDIDATE = "gate_candidate"


def _decay(trust: float, steps_since: int, half_life: int = TRUST_HALF_LIFE_STEPS) -> float:
    if steps_since <= 0:
        return trust
    return trust * (0.5 ** (steps_since / float(half_life)))


@dataclass
class _EdgeMem:
    state: EdgeState = EdgeState.UNKNOWN
    last_step: int = -1
    trust: float = 0.0


@dataclass
class OccupancyMap:
    """The robot's belief.  Independent of the ground-truth DemoMaze."""

    n: int = GRID_N
    step: int = 0
    cells: Dict[Cell, CellState] = field(default_factory=dict)
    cell_last_step: Dict[Cell, int] = field(default_factory=dict)
    edges: Dict[Edge, _EdgeMem] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.geom = GridGeom(self.n)

    # --- queries ---
    def cell_state(self, cell: Cell) -> CellState:
        return self.cells.get(cell, CellState.UNKNOWN)

    def edge_state(self, a: Cell, b: Cell) -> EdgeState:
        mem = self.edges.get(undirected(a, b))
        return mem.state if mem else EdgeState.UNKNOWN

    def is_known_open(self, a: Cell, b: Cell) -> bool:
        return self.edge_state(a, b) in (EdgeState.OPEN, EdgeState.GATE_CANDIDATE)

    def is_known_wall(self, a: Cell, b: Cell) -> bool:
        return self.edge_state(a, b) == EdgeState.WALL

    def edge_trust(self, a: Cell, b: Cell) -> float:
        mem = self.edges.get(undirected(a, b))
        if not mem or mem.state == EdgeState.UNKNOWN:
            return 0.0
        return _decay(mem.trust, self.step - mem.last_step)

    def known_free_cells(self) -> Set[Cell]:
        return {c for c, s in self.cells.items() if s == CellState.FREE}

    # --- updates ---
    def tick(self) -> None:
        self.step += 1

    def mark_free(self, cell: Cell) -> None:
        self.cells[cell] = CellState.FREE
        self.cell_last_step[cell] = self.step

    def _touch_edge(self, a: Cell, b: Cell, state: EdgeState, trust: float = 1.0) -> None:
        key = undirected(a, b)
        self.edges[key] = _EdgeMem(state=state, last_step=self.step, trust=trust)

    def mark_open(self, a: Cell, b: Cell, trust: float = 1.0) -> None:
        self._touch_edge(a, b, EdgeState.OPEN, trust)

    def mark_wall(self, a: Cell, b: Cell, trust: float = 1.0) -> None:
        self._touch_edge(a, b, EdgeState.WALL, trust)

    def mark_gate_candidate(self, a: Cell, b: Cell, trust: float = 1.0) -> None:
        self._touch_edge(a, b, EdgeState.GATE_CANDIDATE, trust)

    def invalidate_edge(self, a: Cell, b: Cell) -> None:
        """Forget an edge after a discrepancy so the planner will not use it."""
        key = undirected(a, b)
        self.edges.pop(key, None)

    def apply_beam(self, cell: Cell, heading: Direction, hit_cells: Optional[float]) -> None:
        """Update the map from one range reading, measured in *cells*.

        `hit_cells` is the distance from the current cell centre to a wall
        face (0.5, 1.5, …) or None if nothing was seen within the useful
        range (treated as "at least 2 cells of free space").
        """
        # Only the cell we are standing in is marked FREE / visited.
        # Distant cells seen down a beam stay UNKNOWN so they remain
        # frontiers until the robot actually steps into them and scans.
        self.mark_free(cell)
        if hit_cells is None:
            # Two cells of free space; the wall beyond is still unknown.
            first = neighbour(cell, heading)
            if self.geom.in_bounds(first):
                self.mark_open(cell, first)
                second = neighbour(first, heading)
                if self.geom.in_bounds(second):
                    self.mark_open(first, second)
            return

        # Walk along the beam and mark open edges until the hit.
        # hit 0.5 → wall on the first edge; hit 1.5 → first edge open, second walled.
        remaining = hit_cells
        cur = cell
        while True:
            nxt = neighbour(cur, heading)
            if remaining <= 1.0:
                # Wall immediately in front of `cur`.
                if self.geom.in_bounds(nxt):
                    self.mark_wall(cur, nxt)
                return
            if not self.geom.in_bounds(nxt):
                return
            self.mark_open(cur, nxt)
            remaining -= 1.0
            cur = nxt

    def refresh_gate_candidates(self) -> None:
        """Tag one-cell openings that sit in a longer wall as gate candidates.

        A 4-adjacent OPEN (or already GATE_CANDIDATE) edge is a candidate
        when the two parallel edges on either side, one cell over, are
        known WALLs — i.e. a single-cell gap in an otherwise solid wall.
        """
        for (a, b) in list(self.edges):
            mem = self.edges[(a, b)]
            if mem.state not in (EdgeState.OPEN, EdgeState.GATE_CANDIDATE):
                continue
            if self._looks_like_gap(a, b):
                # Preserve last_step / trust; only change the label.
                mem.state = EdgeState.GATE_CANDIDATE

    def _looks_like_gap(self, a: Cell, b: Cell) -> bool:
        # a and b differ in exactly one coordinate.
        if a[0] == b[0] and abs(a[1] - b[1]) == 1:
            # Vertical neighbours: the wall runs east–west.  Look at the
            # edges one cell west and one cell east of this door.
            sides = []
            for dx in (-1, 1):
                a2, b2 = (a[0] + dx, a[1]), (b[0] + dx, b[1])
                if self.geom.in_bounds(a2) and self.geom.in_bounds(b2):
                    sides.append(self.edge_state(a2, b2) == EdgeState.WALL)
            return len(sides) == 2 and all(sides)
        if a[1] == b[1] and abs(a[0] - b[0]) == 1:
            sides = []
            for dy in (-1, 1):
                a2, b2 = (a[0], a[1] + dy), (b[0], b[1] + dy)
                if self.geom.in_bounds(a2) and self.geom.in_bounds(b2):
                    sides.append(self.edge_state(a2, b2) == EdgeState.WALL)
            return len(sides) == 2 and all(sides)
        return False

    def reachable_unknown_frontiers(self) -> Set[Cell]:
        """Unknown cells that share a known-open edge with a known-free cell,
        or that are 4-adjacent to a known-free cell through an UNKNOWN edge
        (we have not yet ruled the edge out as a wall).
        """
        frontiers: Set[Cell] = set()
        free = self.known_free_cells()
        for cell in free:
            for _, nxt in self.geom.neighbours(cell):
                if self.is_known_wall(cell, nxt):
                    continue
                if self.cell_state(nxt) != CellState.FREE:
                    frontiers.add(nxt)
        return frontiers

    def is_exploration_complete(self, start: Cell) -> bool:
        """True iff every cell reachable from `start` through known-open
        edges is FREE, and every edge leaving those cells is no longer
        UNKNOWN.  (Unknown cells behind known walls are allowed.)
        """
        if self.cell_state(start) != CellState.FREE:
            return False
        stack = [start]
        seen: Set[Cell] = {start}
        while stack:
            cur = stack.pop()
            for _, nxt in self.geom.neighbours(cur):
                st = self.edge_state(cur, nxt)
                if st == EdgeState.UNKNOWN:
                    return False
                if st == EdgeState.WALL:
                    continue
                if self.cell_state(nxt) != CellState.FREE:
                    return False
                if nxt not in seen:
                    seen.add(nxt)
                    stack.append(nxt)
        return True

    def open_neighbours(self, cell: Cell) -> Iterable[Cell]:
        for _, nxt in self.geom.neighbours(cell):
            if self.is_known_open(cell, nxt) and self.cell_state(nxt) == CellState.FREE:
                yield nxt
