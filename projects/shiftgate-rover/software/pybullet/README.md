# PyBullet simulation

The simulator does not reimplement the mission.  It imports `shiftgate_core`,
runs `Mission` against the stock maze (closing the gate the moment the
package is collected), and then *plays back* the cell-to-cell trace with a
kinematic robot and a TinyRenderer camera.

```bash
python simulate.py --check
python simulate.py --record ../../media/shiftgate_sim.mp4
```

`--check` must print `CHECK OK`.  The MP4 is a **simulation render**, not
real-world footage — the HUD says so on every frame.

Gazebo was not executed when this project was added; the ROS 2 worlds live
in `../ros2_ws/src/shiftgate_gazebo/`.
