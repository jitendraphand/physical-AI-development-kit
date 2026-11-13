"""A* on the witnessed occupancy graph, with trust-weighted edge costs."""

from __future__ import annotations

import heapq
from typing import Dict, List, Optional, Tuple

from .grid import Cell
from .occupancy import OccupancyMap
from .params import TRUST_COST_WEIGHT


class NoPathError(Exception):
    """Raised when the known-open graph has no path between two cells."""


def _manhattan(a: Cell, b: Cell) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def edge_cost(occ: OccupancyMap, a: Cell, b: Cell, use_trust: bool = True) -> float:
    base = 1.0
    if not use_trust:
        return base
    trust = occ.edge_trust(a, b)
    return base + (1.0 - trust) * TRUST_COST_WEIGHT


def astar(
    occ: OccupancyMap,
    start: Cell,
    goal: Cell,
    use_trust: bool = True,
) -> List[Cell]:
    """Return a cell path (including start and goal) of least cost.

    Only edges the map currently believes are OPEN or GATE_CANDIDATE, and
    only cells marked FREE, are traversable.  Ties break toward the
    lexicographically smaller neighbour so tests are deterministic.
    """
    if start == goal:
        return [start]
    if occ.cell_state(start) != occ.cell_state(start):  # pragma: no cover
        pass
    # Allow planning from/to a cell we are standing in even if (e.g. after
    # a reset) it has not been marked; the caller should have marked it.
    open_heap: List[Tuple[float, int, Cell]] = []
    tie = 0
    heapq.heappush(open_heap, (0.0 + _manhattan(start, goal), tie, start))
    came: Dict[Cell, Optional[Cell]] = {start: None}
    gscore: Dict[Cell, float] = {start: 0.0}

    while open_heap:
        _, _, cur = heapq.heappop(open_heap)
        if cur == goal:
            return _reconstruct(came, cur)
        neighbours = sorted(occ.open_neighbours(cur))
        for nxt in neighbours:
            tentative = gscore[cur] + edge_cost(occ, cur, nxt, use_trust=use_trust)
            if nxt not in gscore or tentative < gscore[nxt] - 1e-12:
                came[nxt] = cur
                gscore[nxt] = tentative
                tie += 1
                f = tentative + _manhattan(nxt, goal)
                heapq.heappush(open_heap, (f, tie, nxt))
    raise NoPathError(f"no known path from {start} to {goal}")


def _reconstruct(came: Dict[Cell, Optional[Cell]], cur: Cell) -> List[Cell]:
    path = [cur]
    while came[cur] is not None:
        cur = came[cur]  # type: ignore[assignment]
        path.append(cur)
    path.reverse()
    return path


def path_cost(occ: OccupancyMap, path: List[Cell], use_trust: bool = True) -> float:
    if len(path) <= 1:
        return 0.0
    return sum(edge_cost(occ, a, b, use_trust=use_trust) for a, b in zip(path, path[1:]))
