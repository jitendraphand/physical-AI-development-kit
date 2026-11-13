# ShiftGate Rover — learning curriculum

The robot’s job looks like a school-maze problem.  The skill you are actually practising is **keeping a map honest when the world changes without a notification**.  That is a real Physical AI problem (warehouse aisles, hospital carts, classroom furniture), scaled down to a 5×5 grid and a three-beam sensor.

Each module has a **measurable outcome**, a short exercise, a challenge extension, and a place in the rubric.  You can work through them in order; Modules 4–6 can also be completed entirely in simulation before any PLA is printed.

## Module 1 — CAD and fabrication

**Outcome.**  You have generated every STL from the parametric script, printed the default kit (see [PRINT_SETTINGS.md](PRINT_SETTINGS.md)), and the N20 shaft sits at 24 mm above the table when the chassis is on its wheels, confirmed with the printed `alignment_jig` (go / no-go: the shaft drops into the 3.4 mm hole and is flush with the 24 mm step, within 1 mm).

**Exercise.**  Change `WHEEL_R` (CadQuery) or `WHEEL_RADIUS_M` (params) by 2 mm, re-export the wheel and chassis, and explain in three sentences what else had to stay consistent (shaft height, caster cup, arena ground clearance).

**Challenge.**  Add a parametric cable-clamp feature to `chassis_base` without breaking watertight export (`python cad/verify_stls.py` must still exit 0).

## Module 2 — Electronics and firmware

**Outcome.**  The ESP32 runs the firmware in [firmware/](firmware/shiftgate_firmware/).  With the robot on blocks (wheels off the table):

1. `/cmd_vel` (or the serial equivalent) of `0.08 m/s` for 2.5 s drives both wheels the same number of ticks, ±5 %.
2. On the floor, a 20 cm commanded straight line ends within **3 cm** of the centreline (tape measure, three trials, median counts).
3. The front ultrasonic reports a wall at 10 cm to within **1 cm**, and each side IR, with the pot set for ≈ one cell, flips cleanly at 12–18 cm.

**Exercise.**  Draw the echo-pin voltage divider and measure the echo voltage with a multimeter; it must be ≤ 3.3 V.

**Challenge.**  Close a simple heading loop using only the encoders (no IMU) so a 90° in-place turn ends within 10°.

## Module 3 — ROS 2 and the robot description

**Outcome.**  On Ubuntu 22.04 (Humble) or 24.04 (Jazzy) you can:

```bash
cd software/ros2_ws && colcon build --symlink-install && source install/setup.bash
ros2 launch shiftgate_description rsp.launch.py
ros2 launch shiftgate_gazebo maze.launch.py   # or maze_harmonic.launch.py on Jazzy
```

RViz shows the RobotModel and TF.  Teleop moves the robot in Gazebo.  The URDF wheel separation and wheel radius match `shiftgate_core.params`.

**Exercise.**  Add a TF frame `ultrasonic_link` at the real sensor pose and show the 3-beam scan in RViz.

**Challenge.**  Swap the chassis mesh for the printed STL (`cad/stl/chassis_base.stl`, scaled metres) and confirm collision still works in Gazebo.

## Module 4 — Witnessed occupancy

**Outcome.**  Using `shiftgate_core.OccupancyMap` (or your reimplementation with the same tests):

- A beam of `0.5` cells marks the first edge as `WALL`.
- A beam of `1.5` cells marks the first edge `OPEN` and the second `WALL`.
- Distant cells seen down a beam are **not** marked visited; they remain frontiers.
- After a full exploration of the stock maze, all 25 cells are `FREE` and `is_exploration_complete(start)` is true.
- Trust on an untouched edge halves every `TRUST_HALF_LIFE_STEPS` (40) steps.

**Exercise.**  Run `pytest software/shiftgate_core/tests/test_occupancy.py -q` and then explain, in writing, why marking a far cell as `FREE` without visiting it would hide unknown edges (this was a real bug in an early draft of this project).

**Challenge.**  Tag one-cell gaps in a longer wall as `GATE_CANDIDATE` (`refresh_gate_candidates`) and colour them in the PyBullet HUD or in RViz.

## Module 5 — Planning

**Outcome.**  `pytest software/shiftgate_core/tests/test_planner.py` is green.  In particular, `test_ten_canned_grids_are_optimal` requires that A* match an independent BFS length on ten maps.  On the stock maze, with an oracle map, the open-gate path from `(4,4)` to `(0,0)` has length **8** and uses the gate; the closed-gate path has length **12** and does not.

**Exercise.**  Turn `use_trust=False` and `True` on a map where one of two equal-length first steps is stale; compute both costs by hand and check `path_cost`.

**Challenge.**  Replace 4-connected A* with 8-connected movement *and* a corner-cutting rule (you may not cut a corner if either adjacent edge is a wall).  Extend the tests.

## Module 6 — Discrepancy-triggered replanning

**Outcome.**  `test_full_mission_detects_gate_and_gets_home` is green, and `python software/pybullet/simulate.py --check` prints `CHECK OK`.  Concretely:

- After exploration and fetch, the planned home path uses the gate.
- The world closes the gate with **no message** to the robot.
- The robot raises a `BLOCKED_FREE` discrepancy (either on the first edge or because a planned edge was bricked by a 1.5-cell reading).
- It replans from the cell it is standing in and arrives at `(0,0)`.

**Exercise.**  List the difference between `BLOCKED_FREE` and `OPEN_WALL`.  Invent one sentence for when each should fire in a real corridor.

**Challenge.**  Mid-return, *open* a previously solid wall (the instructor removes a foam panel) and have the robot notice `OPEN_WALL`, then optionally adopt a shorter path if one exists.  Add a unit test.

## Module 7 — Sim-to-real

**Outcome.**  The *same* `Mission` object (or the ROS 2 node that wraps it) runs in PyBullet, in Gazebo, and on the robot.  The only file you change is the launch file / sense function.  On hardware, the robot completes the stock mission, with an instructor-moved gate, in at least **3 of 5** hidden-gate trials (the assessment maze may swap which door is the gate, but a 12-cell backup path must still exist).

**Exercise.**  Time-stamp a log of `MissionEvent`s from sim and from the robot and circle the first discrepancy in each.

**Challenge.**  Introduce a second gate and a “trust decay” so a path the robot has not witnessed for a long time is more expensive than a slightly longer, recently seen corridor.

## Assessment rubric

Score each module 0–3.  A student “passes” the project at **15 / 21**, and “passes with distinction” at **19 / 21** provided Module 6 is at least 2.

| Score | Meaning |
|---|---|
| 0 | Not attempted, or the artefact does not run. |
| 1 | Partial: the idea is visible but a published test or the stated tolerance fails. |
| 2 | Meets the module **Outcome** as written, with the tests / measurements named above. |
| 3 | Outcome plus the **Challenge**, or a clearly argued improvement that still passes the tests. |

Module 6 is the distinctive skill of this project; a 0 there cannot be compensated by a perfect CAD finish.

## Suggested schedule (a 4-week lab, 6–8 hours/week)

| Week | In the lab | At home / on a laptop |
|---|---|---|
| 1 | Print chassis, wheels, clips, gate; cut foam walls | Modules 4–5 in simulation; read the maze ASCII |
| 2 | Assemble, wire, flash; Module 2 straight-line test | Module 3 ROS 2 install; teleop in Gazebo |
| 3 | Build the arena; open-loop drive around one corridor | Module 6 `--check` and watch the simulation render |
| 4 | Full hardware mission, five hidden-gate trials, write-up | Polish the challenge you chose |

## Academic integrity

You may read the provided `shiftgate_core` implementation.  For a *graded* offering, instructors should ask students to reimplement `planner.py` and `discrepancy.py` against the published tests, or to add the Module 6 `OPEN_WALL` challenge with a new test file of their own.
