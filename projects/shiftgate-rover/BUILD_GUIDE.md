# Build guide — from an empty desk to a running mission

Work through these stages in order.  If you only have a laptop, you can complete Stages 0, 6 and 7 first (the algorithm does not need the printer).

## Stage 0 — Laptop software

1. Install Python 3.10 or 3.11 (PyBullet currently publishes wheels up to 3.11).
2. From this folder:

   ```bash
   python -m venv .venv && source .venv/bin/activate
   pip install -r requirements.txt
   pytest -q software/shiftgate_core
   python software/pybullet/simulate.py --check
   ```

   You should see `CHECK OK: explored, detected a silent gate shift, replanned, got home`.
3. Watch [`media/shiftgate_sim.mp4`](media/shiftgate_sim.mp4).  It is a **simulation render**, not real footage.  Notice the moment the orange gate drops and the HUD says `GATE SHIFT DETECTED`.

## Stage 1 — Print

1. (Optional) Re-export the parts so you trust the files:

   ```bash
   python cad/cadquery/generate_parts.py
   python cad/verify_stls.py
   ```

2. Slice every part in [`cad/stl/`](cad/stl/) with the settings in [PRINT_SETTINGS.md](PRINT_SETTINGS.md).  Print the **gate in a contrasting colour**.
3. Default quantities: 1 chassis, 2 wheels, 2 motor clamps, 1 caster cup, 1 US bracket, 2 IR brackets, 1 cover, 1 battery retainer, 1 alignment jig, 1 gate, 24 wall clips, 12 wall posts.  Skip `wall_segment` if you are using foam board.
4. After printing the chassis and wheels, drop an N20 (no wheel yet) into a motor pocket, rest the chassis on the table, and check the shaft against the **alignment jig**.  The shaft should be 24 mm above the table, within 1 mm.  If it is not, add a printed shim or heat-set the motor pocket rather than living with a tilted robot.

## Stage 2 — Cut the arena

1. On a 5 mm foam board, draw a 5×5 grid with **180 mm** inner cells.  The outer frame is ≈ 924 mm (5 × 180 mm + 6 × 4 mm of wall if you also count printed thickness; with foam at 5 mm, ≈ 930 mm).  A 1 m table is enough.
2. Cut strips **176 × 50 mm** (or 180 × 50 mm and trim).  You need the outer perimeter (20 strips) plus the internal walls of the stock maze (count them from the ASCII map in the README — there are 12 internal horizontal walls besides the gate).  Cut a few spares.
3. Push each strip into a `wall_clip` or into the slots of a `wall_post` at T-junctions and corners.  Place the **printed gate** in the centre shaft, between cells `(2,2)` and `(2,3)`, so it can be lifted out and re-inserted mid-mission.
4. Mark the start cell `(0,0)` with green tape and the package cell `(4,4)` with yellow tape.

## Stage 3 — Mechanical assembly

1. Press each wheel onto a motor D-shaft.  Align the flat.  A *light* tap is enough; if it is loose, use a drop of CA glue on the flat only, or the optional M2 set-screw in the hub.
2. Stretch a wide rubber band into each wheel groove.
3. Seat the motors in the side pockets, shafts outwards, wires inwards.  Fit a `motor_clamp` on each side with two M3 screws.  Confirm both shafts still pass the jig.
4. Snap an 8 mm steel ball into the `caster_cup` (the slit lets it pop in) and press or glue the stem up through the front hole of the chassis.
5. Drop the 4×AA holder into the tray and clip the `battery_retainer` over it.  Do not install cells until the wiring has been checked.
6. Screw or zip-tie the HC-SR04 into `sensor_bracket_us` and bolt the bracket to the front bumper (M3, 24 mm spacing).
7. Zip-tie one IR module into each `sensor_bracket_ir`.  Mount them on the left and right side walls, looking sideways, about 20–30 mm back from the front corner.  Set each pot later.
8. Sit the ESP32 and the DRV8833 on the mini breadboard on the front deck (or on the four printed standoffs if you prefer jumper-free headers).  Fit the cover if you want it.

## Stage 4 — Wiring

Follow [hardware/wiring.md](hardware/wiring.md) exactly.  The two rules that save boards:

- **Common ground** between the 4×AA pack and the ESP32.
- **Voltage divider on the HC-SR04 echo pin** (1 kΩ to echo, 2 kΩ to ground, mid-point to GPIO 4).  The trigger pin can be driven at 3.3 V; the echo pin cannot.

Pin map (ESP32 DevKit V1, avoiding strapping and input-only mistakes):

| Function | GPIO |
|---|---|
| Motor left  AIN1 / AIN2 | 26 / 27 |
| Motor right BIN1 / BIN2 | 32 / 33 |
| DRV8833 nSLEEP | 25 (or tie to 3.3 V) |
| Encoder left C1 / C2 | 16 / 17 |
| Encoder right C1 / C2 | 18 / 19 |
| Encoder VCC | 3.3 V |
| HC-SR04 TRIG / ECHO | 23 / 4 (echo via divider) |
| HC-SR04 VCC | 5 V (USB 5 V pin) |
| IR left / right OUT | 34 / 35 (input-only, fine) |
| I²C SDA / SCL (optional IMU) | 21 / 22 |

## Stage 5 — Flash the firmware

1. Install [PlatformIO](https://platformio.org/) (VS Code extension or `pip install platformio`).
2. Connect the ESP32 over USB-C, hold BOOT if it does not enter download mode.

   ```bash
   cd firmware/shiftgate_firmware
   pio run -t upload
   pio device monitor -b 115200
   ```

3. You should see encoder ticks change when you turn a wheel by hand, and a range in centimetres when you put a notebook in front of the ultrasonic.
4. For micro-ROS: build with `-DSHIFTGATE_MICROROS=1` once the micro-ROS Arduino component is installed, then on the laptop:

   ```bash
   # Humble example
   source /opt/ros/humble/setup.bash
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
   ```

   The serial fallback (default) talks a simple line protocol so you can debug without an agent.  See [firmware/shiftgate_firmware/README.md](firmware/shiftgate_firmware/README.md).

5. Run the Module 2 tests (20 cm straight line, IR pots).  Only then put the robot in the maze.

## Stage 6 — ROS 2 workspace

On Ubuntu 22.04 (Humble) or 24.04 (Jazzy):

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd software/ros2_ws
rosdep install --from-paths src --ignore-src -y   # if you use rosdep
colcon build --symlink-install
source install/setup.bash
ros2 launch shiftgate_description rsp.launch.py
# in another terminal, after sourcing:
ros2 launch shiftgate_gazebo maze.launch.py          # Humble, Gazebo Classic
# or
ros2 launch shiftgate_gazebo maze_harmonic.launch.py # Jazzy, Gazebo Harmonic
```

The nav node:

```bash
ros2 launch shiftgate_bringup mission.launch.py use_sim:=true
```

On the robot, set `use_sim:=false` and make sure the micro-ROS agent is running.

## Stage 7 — The mission, on the table

1. Place the robot in cell `(0,0)`, heading east (along the first corridor).  Gate **open**.
2. Start the mission.  The robot should visit every cell (frontier exploration) and then drive to the yellow package cell.
3. The moment it reaches `(4,4)`, the instructor **slides the gate shut**.  Do not tell the robot, do not publish a message, do not restart the node.
4. Watch for a discrepancy event (`BLOCKED_FREE`) when it meets the closed gate, then a replan, then a return along the staggered-door detour.
5. Success: the robot stops in `(0,0)` and the event log contains `discrepancy`, `replan`, and `home`.

Repeat with the gate left open (should take the 8-cell shaft) and with the gate pre-closed before fetch (should never plan the shaft).  Those two checks catch a surprising number of bugs.

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| ESP32 resets when the motors start | Motor supply and ESP32 share a weak source, or ground is missing.  Use the 4×AA pack for motors and USB for the ESP32, common GND. |
| Echo always 0 / always max | Divider wired backwards, or object inside the 2 cm blind zone. |
| IR always triggers | Pot turned too far; sunlight; shiny floor.  Recalibrate in the maze lighting. |
| Robot curves on a straight command | One motor clamp is tight, or rubber bands don’t match.  Swap bands, retake the 20 cm test. |
| Map is complete but the robot skips a cell | You marked far cells as visited from a long beam.  Distant cells must stay frontiers (Module 4 exercise). |
| No discrepancy when the gate closes | The planned path was not invalidated after a 1.5-cell reading.  The provided `Mission._replan_if_plan_broken` exists for this. |
