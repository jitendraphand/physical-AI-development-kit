"""Frontier exploration on the occupancy map.

The explorer is deliberately simple: walk to the nearest frontier (BFS on
the known-open graph, then one step across the unknown edge).  Sensing is
the caller's job — typically a 3-beam scan (left IR, front ultrasonic,
right IR) plus an optional in-place 4-heading scan when a cell is first
visited.  This keeps the logic testable without a physics engine.
"""

from __future__ import annotations

from collections import deque
from typing import List, Optional, Tuple

from .grid import Cell, Direction, neighbour
from .occupancy import OccupancyMap
from .planner import NoPathError, astar


class FrontierExplorer:
    def __init__(self, occ: OccupancyMap, start: Cell, heading: Direction = Direction.EAST):
        self.occ = occ
        self.cell = start
        self.heading = heading
        self.occ.mark_free(start)

    def next_target(self) -> Optional[Cell]:
        """Nearest frontier cell, or None if exploration looks complete."""
        if self.occ.is_exploration_complete(self.cell) and not self.occ.reachable_unknown_frontiers():
            return None
        frontiers = self.occ.reachable_unknown_frontiers()
        if not frontiers:
            return None
        # BFS from the current cell over known-open edges, plus a single
        # step into a frontier across a non-wall edge.
        q = deque([self.cell])
        seen = {self.cell}
        while q:
            cur = q.popleft()
            # Prefer a deterministic neighbour order.
            for _, nxt in sorted(self.occ.geom.neighbours(cur), key=lambda t: t[1]):
                if self.occ.is_known_wall(cur, nxt):
                    continue
                if nxt in seen:
                    continue
                if nxt in frontiers:
                    return nxt
                if self.occ.is_known_open(cur, nxt):
                    seen.add(nxt)
                    q.append(nxt)
        # A frontier exists but is not reachable through current beliefs;
        # the mission should then declare exploration stuck.
        return None

    def path_to(self, target: Cell) -> List[Cell]:
        """Path to a known-free cell, or to a frontier adjacent to known free."""
        if self.occ.cell_state(target) == self.occ.cell_state(target) and target in self.occ.known_free_cells():
            return astar(self.occ, self.cell, target, use_trust=False)
        # Walk to a known-free neighbour of the frontier, then step in.
        for _, nbr in sorted(self.occ.geom.neighbours(target), key=lambda t: t[1]):
            if nbr in self.occ.known_free_cells() and not self.occ.is_known_wall(nbr, target):
                prefix = astar(self.occ, self.cell, nbr, use_trust=False)
                return prefix + [target]
        raise NoPathError(f"cannot reach frontier {target} from {self.cell}")

    def turn_toward(self, target: Cell) -> Direction:
        dx, dy = target[0] - self.cell[0], target[1] - self.cell[1]
        if abs(dx) >= abs(dy):
            return Direction.EAST if dx > 0 else Direction.WEST
        return Direction.NORTH if dy > 0 else Direction.SOUTH
