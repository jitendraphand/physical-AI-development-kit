"""Classify a fresh sensor reading against the current map belief."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Optional

from .grid import Cell, Direction, neighbour
from .occupancy import EdgeState, OccupancyMap


class DiscrepancyKind(Enum):
    BLOCKED_FREE = "blocked_free"  # planned / believed-open edge is now a wall
    OPEN_WALL = "open_wall"  # believed-wall edge is now open
    # A cell we have never committed to is fine; that is ordinary sensing.
    NONE = "none"


@dataclass(frozen=True)
class Discrepancy:
    kind: DiscrepancyKind
    cell: Cell  # cell the robot was in when it sensed
    heading: Direction
    believed: EdgeState
    observed_hit_cells: Optional[float]

    @property
    def edge_next(self) -> Optional[Cell]:
        return neighbour(self.cell, self.heading)


def classify_observation(
    occ: OccupancyMap,
    cell: Cell,
    heading: Direction,
    hit_cells: Optional[float],
) -> Discrepancy:
    """Compare one beam to the map.

    A hit of 0.5 cells means the edge immediately in front is a wall.  A
    hit of 1.5+ or a miss (None, meaning ≥ 2 cells clear) means that first
    edge is open.  Only the *first* edge is used for discrepancy; deeper
    cells are handled by the ordinary occupancy update.
    """
    nxt = neighbour(cell, heading)
    believed = (
        occ.edge_state(cell, nxt) if occ.geom.in_bounds(nxt) else EdgeState.WALL
    )
    # Observation of the first edge.
    first_is_wall = hit_cells is not None and hit_cells <= 1.0

    if believed in (EdgeState.OPEN, EdgeState.GATE_CANDIDATE) and first_is_wall:
        return Discrepancy(DiscrepancyKind.BLOCKED_FREE, cell, heading, believed, hit_cells)
    if believed == EdgeState.WALL and not first_is_wall:
        # An "open wall" is only flagged when we can actually step into a
        # neighbouring cell (not when we are staring at the outer boundary).
        if occ.geom.in_bounds(nxt):
            return Discrepancy(DiscrepancyKind.OPEN_WALL, cell, heading, believed, hit_cells)
    return Discrepancy(DiscrepancyKind.NONE, cell, heading, believed, hit_cells)
