from shiftgate_core.maze import ascii_maze, demo_maze
from shiftgate_core.mission import Mission, MissionConfig, MissionPhase, sense_from_maze
from shiftgate_core.occupancy import OccupancyMap
from shiftgate_core.planner import astar


def test_demo_maze_ascii_renders():
    maze = demo_maze()
    text = ascii_maze(maze)
    assert "S" in text and "P" in text
    # Gate starts open, drawn as spaces between === markers' position;
    # the closed gate uses '='.
    maze.close_gate()
    closed = ascii_maze(maze)
    assert "===" in closed


def test_shortest_path_uses_gate_when_open():
    maze = demo_maze()
    occ = OccupancyMap(n=maze.n)
    # Oracle map: every true opening is known.
    for cell in occ.geom.cells():
        occ.mark_free(cell)
    for a, b in occ.geom.all_edges():
        if maze.is_blocked(a, b):
            occ.mark_wall(a, b)
        else:
            occ.mark_open(a, b)
    open_path = astar(occ, maze.package, maze.start, use_trust=False)
    assert len(open_path) - 1 == 8
    # The critical gate edge must be on every optimal path.
    edges = list(zip(open_path, open_path[1:]))
    gate = maze.gate
    assert gate in [(a, b) if a <= b else (b, a) for a, b in edges]

    maze.close_gate()
    occ.mark_wall(*maze.gate)
    closed_path = astar(occ, maze.package, maze.start, use_trust=False)
    assert len(closed_path) - 1 == 12
    closed_edges = [(a, b) if a <= b else (b, a) for a, b in zip(closed_path, closed_path[1:])]
    assert gate not in closed_edges


def test_full_mission_detects_gate_and_gets_home():
    maze = demo_maze()
    occ = OccupancyMap(n=maze.n)
    mission = Mission(
        occ=occ,
        sense=sense_from_maze(maze),
        config=MissionConfig(start=maze.start, package=maze.package),
    )

    # Explore + fetch, but do not yet return past the gate.
    # Run until the robot is on the RETURN path and has stepped once,
    # then close the gate.
    steps = 0
    while mission.phase not in (MissionPhase.RETURN, MissionPhase.FAILED, MissionPhase.DONE) and steps < 400:
        mission.step()
        steps += 1
    assert mission.phase == MissionPhase.RETURN, f"never started return after {steps} steps: {mission.events[-5:]}"
    assert mission.cell == maze.package or mission.return_steps >= 0

    # Close the gate once the robot has left the package cell and is
    # still north of the gate (y >= 3), so it will meet the closed door.
    closed = False
    for _ in range(400):
        if (
            not closed
            and mission.phase == MissionPhase.RETURN
            and mission.cell[1] >= 3
            and mission.return_steps >= 1
        ):
            maze.close_gate()
            closed = True
        phase = mission.step()
        if phase in (MissionPhase.DONE, MissionPhase.FAILED):
            break

    kinds = [e.kind for e in mission.events]
    assert "discrepancy" in kinds, f"no discrepancy logged: {mission.events}"
    assert any("BLOCKED_FREE" in e.detail for e in mission.events)
    assert "replan" in kinds
    assert mission.phase == MissionPhase.DONE
    assert mission.cell == maze.start


def test_mission_without_gate_change_still_gets_home():
    maze = demo_maze()
    occ = OccupancyMap(n=maze.n)
    mission = Mission(occ, sense_from_maze(maze), MissionConfig(start=maze.start, package=maze.package))
    phase = mission.run(max_steps=400)
    assert phase == MissionPhase.DONE
    assert mission.cell == maze.start
    # Map should be complete and every cell visited.
    assert occ.is_exploration_complete(maze.start)
    assert len(occ.known_free_cells()) == 25
