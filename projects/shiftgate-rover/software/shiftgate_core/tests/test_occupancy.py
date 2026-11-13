from shiftgate_core.grid import Direction
from shiftgate_core.occupancy import CellState, EdgeState, OccupancyMap


def test_beam_hit_half_marks_wall_on_first_edge():
    occ = OccupancyMap(n=5)
    occ.mark_free((1, 1))
    occ.apply_beam((1, 1), Direction.EAST, 0.5)
    assert occ.edge_state((1, 1), (2, 1)) == EdgeState.WALL
    assert occ.cell_state((2, 1)) == CellState.UNKNOWN


def test_beam_hit_one_and_half_marks_open_then_wall():
    occ = OccupancyMap(n=5)
    occ.apply_beam((1, 1), Direction.EAST, 1.5)
    assert occ.edge_state((1, 1), (2, 1)) in (EdgeState.OPEN, EdgeState.GATE_CANDIDATE)
    # Distant cells are seen, not visited.
    assert occ.cell_state((2, 1)) == CellState.UNKNOWN
    assert occ.edge_state((2, 1), (3, 1)) == EdgeState.WALL


def test_beam_miss_marks_two_open_edges_without_visiting():
    occ = OccupancyMap(n=5)
    occ.apply_beam((0, 0), Direction.EAST, None)
    assert occ.cell_state((0, 0)) == CellState.FREE
    assert occ.cell_state((1, 0)) == CellState.UNKNOWN
    assert occ.cell_state((2, 0)) == CellState.UNKNOWN
    assert occ.edge_state((0, 0), (1, 0)) in (EdgeState.OPEN, EdgeState.GATE_CANDIDATE)
    assert occ.edge_state((1, 0), (2, 0)) in (EdgeState.OPEN, EdgeState.GATE_CANDIDATE)


def test_gate_candidate_is_a_one_cell_gap():
    occ = OccupancyMap(n=5)
    # A horizontal wall (north of row y=1) with a single gap at x=2.
    for x in range(5):
        a, b = (x, 1), (x, 2)
        if x == 2:
            occ.mark_open(a, b)
        else:
            occ.mark_wall(a, b)
    occ.refresh_gate_candidates()
    assert occ.edge_state((2, 1), (2, 2)) == EdgeState.GATE_CANDIDATE
    assert occ.edge_state((1, 1), (1, 2)) == EdgeState.WALL


def test_trust_decays_with_steps():
    occ = OccupancyMap(n=3)
    occ.mark_open((0, 0), (1, 0), trust=1.0)
    t0 = occ.edge_trust((0, 0), (1, 0))
    occ.step += 40  # one half-life
    t1 = occ.edge_trust((0, 0), (1, 0))
    assert abs(t1 - 0.5 * t0) < 1e-9
