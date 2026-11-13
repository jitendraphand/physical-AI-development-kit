"""Grid geometry helpers: cells, edges, and cardinal headings."""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from typing import Iterable, Iterator, Tuple

Cell = Tuple[int, int]
# An undirected edge is stored with the lexicographically smaller cell first.
Edge = Tuple[Cell, Cell]


class Direction(IntEnum):
    EAST = 0
    NORTH = 1
    WEST = 2
    SOUTH = 3

    def left(self) -> "Direction":
        return Direction((int(self) + 1) % 4)

    def right(self) -> "Direction":
        return Direction((int(self) - 1) % 4)

    def opposite(self) -> "Direction":
        return Direction((int(self) + 2) % 4)


DELTA = {
    Direction.EAST: (1, 0),
    Direction.NORTH: (0, 1),
    Direction.WEST: (-1, 0),
    Direction.SOUTH: (0, -1),
}


def heading_to_delta(heading: Direction) -> Cell:
    return DELTA[heading]


def neighbour(cell: Cell, heading: Direction) -> Cell:
    dx, dy = DELTA[heading]
    return (cell[0] + dx, cell[1] + dy)


def undirected(a: Cell, b: Cell) -> Edge:
    return (a, b) if a <= b else (b, a)


def direction_from(a: Cell, b: Cell) -> Direction:
    dx, dy = b[0] - a[0], b[1] - a[1]
    if (dx, dy) == (1, 0):
        return Direction.EAST
    if (dx, dy) == (-1, 0):
        return Direction.WEST
    if (dx, dy) == (0, 1):
        return Direction.NORTH
    if (dx, dy) == (0, -1):
        return Direction.SOUTH
    raise ValueError(f"{a} and {b} are not 4-adjacent")


@dataclass(frozen=True)
class GridGeom:
    n: int

    def in_bounds(self, cell: Cell) -> bool:
        x, y = cell
        return 0 <= x < self.n and 0 <= y < self.n

    def cells(self) -> Iterator[Cell]:
        for y in range(self.n):
            for x in range(self.n):
                yield (x, y)

    def neighbours(self, cell: Cell) -> Iterator[Tuple[Direction, Cell]]:
        for heading in Direction:
            nxt = neighbour(cell, heading)
            if self.in_bounds(nxt):
                yield heading, nxt

    def all_edges(self) -> Iterable[Edge]:
        seen = set()
        for cell in self.cells():
            for _, nxt in self.neighbours(cell):
                e = undirected(cell, nxt)
                if e not in seen:
                    seen.add(e)
                    yield e
