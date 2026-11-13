"""High-level mission: explore, fetch the package, return home, replan.

The Mission object is the single source of truth used by the PyBullet
demo, the ROS 2 node, and the unit tests.  It does not move motors; it
consumes abstract observations (`sense(cell, heading) -> hit_cells`) and
emits the next cell to stand in plus events the HUD can display.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Callable, List, Optional

from .discrepancy import Discrepancy, DiscrepancyKind, classify_observation
from .explore import FrontierExplorer
from .grid import Cell, Direction, direction_from, neighbour
from .maze import DemoMaze
from .occupancy import OccupancyMap
from .planner import NoPathError, astar


class MissionPhase(Enum):
    EXPLORE = "explore"
    FETCH = "fetch"
    RETURN = "return"
    REEXPLORE = "reexplore"
    DONE = "done"
    FAILED = "failed"


@dataclass
class MissionEvent:
    step: int
    kind: str
    detail: str


@dataclass
class MissionConfig:
    start: Cell = (0, 0)
    package: Cell = (4, 4)
    # After this many RETURN steps, the caller is expected to have closed
    # the gate.  The mission itself does not close it — the world does.
    # Used only as documentation for the stock demo; not enforced here.
    gate_closes_after_return_steps: int = 2
    scan_four_ways_on_entry: bool = True


SenseFn = Callable[[Cell, Direction], Optional[float]]


@dataclass
class Mission:
    occ: OccupancyMap
    sense: SenseFn
    config: MissionConfig = field(default_factory=MissionConfig)
    heading: Direction = Direction.EAST

    def __post_init__(self) -> None:
        self.phase = MissionPhase.EXPLORE
        self.cell: Cell = self.config.start
        self.events: List[MissionEvent] = []
        self.path: List[Cell] = []
        self.explorer = FrontierExplorer(self.occ, self.config.start, self.heading)
        self.return_steps = 0
        self.last_discrepancy: Optional[Discrepancy] = None
        self.occ.mark_free(self.cell)
        self._log("start", f"begin explore at {self.cell}")
        self._scan_here()

    def _log(self, kind: str, detail: str) -> None:
        self.events.append(MissionEvent(self.occ.step, kind, detail))

    def _scan_here(self) -> Optional[Discrepancy]:
        """Sense.  Optionally scan four headings the first time we enter a cell."""
        self.occ.tick()
        found: Optional[Discrepancy] = None
        headings = (
            [self.heading, self.heading.left(), self.heading.right(), self.heading.opposite()]
            if self.config.scan_four_ways_on_entry
            else [self.heading, self.heading.left(), self.heading.right()]
        )
        # De-duplicate while preserving order.
        seen = set()
        unique = []
        for h in headings:
            if h not in seen:
                seen.add(h)
                unique.append(h)
        for h in unique:
            hit = self.sense(self.cell, h)
            disc = classify_observation(self.occ, self.cell, h, hit)
            self.occ.apply_beam(self.cell, h, hit)
            if disc.kind != DiscrepancyKind.NONE and found is None:
                found = disc
        self.occ.refresh_gate_candidates()
        return found

    def _apply_discrepancy(self, disc: Discrepancy) -> None:
        self.last_discrepancy = disc
        nxt = disc.edge_next
        if disc.kind == DiscrepancyKind.BLOCKED_FREE and nxt is not None:
            self.occ.mark_wall(self.cell, nxt)
            self._log("discrepancy", f"BLOCKED_FREE in front of {self.cell} heading {disc.heading.name}")
        elif disc.kind == DiscrepancyKind.OPEN_WALL and nxt is not None:
            self.occ.mark_open(self.cell, nxt)
            self._log("discrepancy", f"OPEN_WALL in front of {self.cell} heading {disc.heading.name}")

    def _replan_home(self) -> bool:
        try:
            self.path = astar(self.occ, self.cell, self.config.start, use_trust=True)
            self.phase = MissionPhase.RETURN
            self._log("replan", f"new home path {self.path}")
            return True
        except NoPathError:
            self.phase = MissionPhase.REEXPLORE
            self.path = []
            self._log("replan_failed", "no path home; falling back to re-exploration")
            return False

    def _blocked_planned_edge(self):
        """Return the first planned edge that the map now believes is a wall."""
        if not self.path:
            return None
        cells = self.path if self.path[0] == self.cell else (
            [self.cell] + self.path if self.cell not in self.path else self.path[self.path.index(self.cell):]
        )
        for a, b in zip(cells, cells[1:]):
            if self.occ.is_known_wall(a, b):
                return a, b
        return None

    def _replan_if_plan_broken(self, goal: Cell, log_discrepancy: bool = True) -> bool:
        """If a scan has bricked an edge on the current plan, replan.

        Seeing a new wall one cell ahead (range 1.5) updates the map without
        the *first-edge* classifier firing.  This check is what actually
        fires the discrepancy in that case.
        """
        br = self._blocked_planned_edge()
        if br is None:
            return False
        a, b = br
        if log_discrepancy:
            self._log("discrepancy", f"BLOCKED_FREE planned edge {a}--{b} at cell {self.cell}")
        if goal == self.config.start:
            self._replan_home()
        else:
            try:
                self.path = astar(self.occ, self.cell, goal, use_trust=True)
                self._log("replan", f"new path to {goal}: {self.path}")
            except NoPathError:
                self.path = []
                self.phase = MissionPhase.FAILED
                self._log("fail", f"no path to {goal} after blocked edge {a}--{b}")
        return True

    def _advance_along_path(self) -> bool:
        if not self.path or self.path[0] != self.cell:
            # Re-sync: if we are on the path, trim; else abort.
            if self.cell in self.path:
                i = self.path.index(self.cell)
                self.path = self.path[i:]
            else:
                return False
        if len(self.path) == 1:
            return True  # already at the end
        nxt = self.path[1]
        self.heading = direction_from(self.cell, nxt)
        self.cell = nxt
        self.explorer.cell = self.cell
        self.path = self.path[1:]
        return True

    def step(self) -> MissionPhase:
        """Advance one cell (or one decision).  Returns the new phase."""
        if self.phase in (MissionPhase.DONE, MissionPhase.FAILED):
            return self.phase

        if self.phase == MissionPhase.EXPLORE:
            if self.occ.is_exploration_complete(self.config.start):
                self._log("explored", "map complete")
                self.phase = MissionPhase.FETCH
                try:
                    self.path = astar(self.occ, self.cell, self.config.package, use_trust=True)
                    self._log("fetch_plan", f"path to package {self.path}")
                except NoPathError:
                    self.phase = MissionPhase.FAILED
                    self._log("fail", "no path to package after exploration")
                return self.phase
            target = self.explorer.next_target()
            if target is None:
                # No frontier but completeness check failed — stuck.
                self.phase = MissionPhase.FAILED
                self._log("fail", "exploration stuck with unknown edges")
                return self.phase
            try:
                self.path = self.explorer.path_to(target)
            except NoPathError:
                self.phase = MissionPhase.FAILED
                self._log("fail", f"cannot reach frontier {target}")
                return self.phase
            if len(self.path) >= 2:
                self._advance_along_path()
            disc = self._scan_here()
            self.explorer.cell = self.cell
            if disc and disc.kind != DiscrepancyKind.NONE:
                self._apply_discrepancy(disc)
            return self.phase

        if self.phase == MissionPhase.FETCH:
            if self.cell == self.config.package:
                self._log("fetched", "package reached; planning home")
                if not self._replan_home():
                    return self.phase
                return self.phase
            if not self.path or len(self.path) < 2:
                try:
                    self.path = astar(self.occ, self.cell, self.config.package, use_trust=True)
                except NoPathError:
                    self.phase = MissionPhase.FAILED
                    self._log("fail", "lost path to package")
                    return self.phase
            self._advance_along_path()
            disc = self._scan_here()
            if disc and disc.kind != DiscrepancyKind.NONE:
                self._apply_discrepancy(disc)
                try:
                    self.path = astar(self.occ, self.cell, self.config.package, use_trust=True)
                except NoPathError:
                    self.phase = MissionPhase.FAILED
                return self.phase
            self._replan_if_plan_broken(self.config.package)
            return self.phase

        if self.phase == MissionPhase.RETURN:
            if self.cell == self.config.start:
                self.phase = MissionPhase.DONE
                self._log("home", "returned to start")
                return self.phase
            if not self.path or len(self.path) < 2:
                if not self._replan_home():
                    return self.phase
            # Look *ahead* at the next edge before rolling onto it, so a
            # closed gate is seen from the cell in front of it.
            nxt = self.path[1]
            self.heading = direction_from(self.cell, nxt)
            hit = self.sense(self.cell, self.heading)
            disc = classify_observation(self.occ, self.cell, self.heading, hit)
            self.occ.tick()
            self.occ.apply_beam(self.cell, self.heading, hit)
            if disc.kind != DiscrepancyKind.NONE:
                self._apply_discrepancy(disc)
                if not self._replan_home():
                    return self.phase
                return self.phase
            if self._replan_if_plan_broken(self.config.start):
                return self.phase
            self._advance_along_path()
            self.return_steps += 1
            disc = self._scan_here()
            if disc and disc.kind != DiscrepancyKind.NONE:
                self._apply_discrepancy(disc)
                self._replan_home()
                return self.phase
            if self._replan_if_plan_broken(self.config.start):
                return self.phase
            return self.phase

        if self.phase == MissionPhase.REEXPLORE:
            # Limited: walk to the nearest frontier; if none, fail.
            target = self.explorer.next_target()
            if target is None:
                # Maybe a path home appeared after the last scan.
                if self._replan_home():
                    return self.phase
                self.phase = MissionPhase.FAILED
                self._log("fail", "re-exploration found no path home")
                return self.phase
            try:
                self.path = self.explorer.path_to(target)
            except NoPathError:
                self.phase = MissionPhase.FAILED
                return self.phase
            if len(self.path) >= 2:
                self._advance_along_path()
            self._scan_here()
            # After every re-explore step, try to go home again.
            self._replan_home()
            return self.phase

        return self.phase

    def run(self, max_steps: int = 400) -> MissionPhase:
        for _ in range(max_steps):
            self.step()
            if self.phase in (MissionPhase.DONE, MissionPhase.FAILED):
                return self.phase
        self.phase = MissionPhase.FAILED
        self._log("fail", "exceeded max_steps")
        return self.phase


def sense_from_maze(maze: DemoMaze) -> SenseFn:
    """Build a SenseFn that reads the ground-truth maze (for tests / sims)."""

    def _sense(cell: Cell, heading: Direction) -> Optional[float]:
        return maze.sense_range(cell, heading, max_cells=2)

    return _sense
