# ShiftGate ESP32 firmware

PlatformIO project, Arduino framework, board `esp32dev`.

```bash
pio run -t upload
pio device monitor -b 115200
```

## Serial protocol (default build)

| You type | Meaning |
|---|---|
| `v 0.08 0.0` | Linear 0.08 m/s, angular 0 (straight). |
| `v 0.0 0.8` | In-place turn, +0.8 rad/s. |
| `stop` | Coast to zero. |
| `zero` | Reset odometry to the origin. |

The board prints a `st ...` telemetry line at 20 Hz: pose, tick counts, ultrasonic metres, and IR flags (1 = obstacle).

## micro-ROS (optional)

1. Install the [micro-ROS Arduino](https://github.com/micro-ROS/micro_ros_arduino) component into `lib/` (or as a lib_dep) following their ESP32 + PlatformIO instructions for your ROS 2 distro.
2. Uncomment `-DSHIFTGATE_MICROROS=1` in `platformio.ini` and extend `main.cpp` with the standard `rclc` publisher / subscriber pattern (the pin map and `set_cmd_vel` / `integrate_odom` helpers stay as they are).
3. On the laptop: `ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0`.

A serial-bridge node in `shiftgate_bringup` can also parse the `st` lines if you would rather not install micro-ROS on day one.

## Safety

`nSLEEP` is driven HIGH in `setup()`.  If USB drops, the sketch stops calling `set_cmd_vel` but the last PWM may remain — the **hardware SPST switch** on the 4×AA pack is the real e-stop.  Use it.
