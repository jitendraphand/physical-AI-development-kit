# Physical AI Development Kit

Hands-on robotics and Physical AI projects you can **build yourself**: 3D-printed parts, a cost-friendly Indian bill of materials, step-by-step assembly, and the same industry tools used in labs — **ROS 2**, **FreeCAD** / **CadQuery**, **Gazebo**, and **PyBullet**.

This repository is a *kit*: each folder under [`projects/`](projects/) is a complete, self-contained robot with its own curriculum, CAD, firmware, and simulation.  Nothing here requires a prior project.

| Project | What you build | Distinctive skill | Target BOM (India) |
|---|---|---|---|
| [**ShiftGate Rover**](projects/shiftgate-rover/) | A palm-sized ESP32 + micro-ROS differential-drive rover and a modular 5×5 tabletop maze | *Witnessed occupancy* and **discrepancy-triggered replanning** after a silent gate shift | ≈ **₹2,400–2,700** (under ₹3,500) |

## Who this is for

Students and instructors who want a first (or second) physical robot that goes beyond “follow a line” or “solve a known maze”.  The tasks are small; the *reasoning* is the point.

## Tooling philosophy

- **CAD is parametric.**  Every printed part is generated from a script (CadQuery, with a FreeCAD variant) so you can change a number and re-export STL / STEP.
- **The algorithm is shared.**  Mapping, planning, and replanning live in a pure-Python library used by unit tests, the PyBullet demo, and the ROS 2 nodes.  You can learn the logic on a laptop before touching a soldering iron.
- **Simulation is honest.**  Videos in this repo are labelled as **simulation renders**, not camera footage of a real robot.
- **Cost is a first-class constraint.**  Parts are chosen so a student in India can order them from Robocraze, Robu / RobuKits, Circuit City, or a local hardware shop.

## Licence

MIT.  See [LICENSE](LICENSE).

## Contributing

New projects should follow the ShiftGate layout: a self-contained `projects/<name>/` with CAD, BOM (verified India links), a learning curriculum with measurable outcomes, tests, and at least one simulation.  Open a pull request; do not rewrite existing projects unless you are fixing them.
