from shiftgate_core.discrepancy import DiscrepancyKind, classify_observation
from shiftgate_core.grid import Direction
from shiftgate_core.occupancy import OccupancyMap


def test_blocked_free_when_open_edge_is_now_a_wall():
    occ = OccupancyMap(n=5)
    occ.mark_free((2, 2))
    occ.mark_free((2, 3))
    occ.mark_open((2, 2), (2, 3))
    disc = classify_observation(occ, (2, 2), Direction.NORTH, 0.5)
    assert disc.kind == DiscrepancyKind.BLOCKED_FREE


def test_open_wall_when_believed_wall_is_clear():
    occ = OccupancyMap(n=5)
    occ.mark_free((2, 2))
    occ.mark_wall((2, 2), (2, 3))
    disc = classify_observation(occ, (2, 2), Direction.NORTH, 1.5)
    assert disc.kind == DiscrepancyKind.OPEN_WALL


def test_no_discrepancy_when_observation_matches():
    occ = OccupancyMap(n=5)
    occ.mark_open((2, 2), (2, 3))
    disc = classify_observation(occ, (2, 2), Direction.NORTH, 1.5)
    assert disc.kind == DiscrepancyKind.NONE


def test_unknown_edge_is_not_a_discrepancy():
    occ = OccupancyMap(n=5)
    disc = classify_observation(occ, (0, 0), Direction.EAST, 0.5)
    assert disc.kind == DiscrepancyKind.NONE
