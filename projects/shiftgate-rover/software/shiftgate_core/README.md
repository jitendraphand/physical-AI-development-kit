# shiftgate_core

Pure-Python library shared by the PyBullet simulator, the unit tests, and the
ROS 2 nodes. There is **no ROS or PyBullet dependency** here, so you can
reason about the algorithm on a laptop before touching a robot.

## What is distinctive

The task looks like "solve a maze and come home". The actual skill is
**witnessed occupancy** plus **discrepancy-triggered replanning (DTR)**:

1. While exploring, every cell and every edge stores *what* was seen and
   *when* it was last confirmed (a trust score that decays).
2. After a complete map is built, A* plans a shortest return, preferring
   recently witnessed free edges.
3. The world is allowed to change *without a notification* (a gate tile is
   moved). The robot must classify the mismatch
   (`BLOCKED_FREE` vs `OPEN_WALL`) and replan from its current cell.
4. If the updated map has no path home, the robot falls back to a limited
   re-exploration of frontiers that touch the broken edge.

See the project [LEARNING.md](../../LEARNING.md) for measurable outcomes.
