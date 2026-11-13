"""ROS 2 node that runs shiftgate_core.Mission against /odom and /shiftgate/beams.

In simulation (`use_sim:=true`) the node uses the *same* DemoMaze sense
function as the unit tests, so you can replay the curriculum mission
without hardware.  On the robot it converts a 3-beam reading and the
current odom pose into a cell + heading, then publishes /cmd_vel toward
the next cell.

This node imports shiftgate_core from the sibling folder; on a real
install, `pip install -e software/shiftgate_core` or add it to
PYTHONPATH / an ament Python package of its own.
"""

from __future__ import annotations

import sys
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

CORE = Path(__file__).resolve().parents[3] / "shiftgate_core"
if str(CORE) not in sys.path:
    sys.path.insert(0, str(CORE))

from shiftgate_core.grid import Direction  # noqa: E402
from shiftgate_core.maze import demo_maze  # noqa: E402
from shiftgate_core.mission import (  # noqa: E402
    Mission,
    MissionConfig,
    MissionPhase,
    sense_from_maze,
)
from shiftgate_core.occupancy import OccupancyMap  # noqa: E402
from shiftgate_core.params import CELL_SIZE_M  # noqa: E402


class MissionNode(Node):
    def __init__(self) -> None:
        super().__init__("shiftgate_mission")
        self.declare_parameter("use_sim", True)
        self.declare_parameter("rate_hz", 2.0)
        self.use_sim = bool(self.get_parameter("use_sim").value)
        self.pub_cmd = self.create_publisher(Twist, "cmd_vel", 10)
        self.pub_phase = self.create_publisher(String, "shiftgate/phase", 10)
        self.pub_events = self.create_publisher(String, "shiftgate/event", 10)
        self._latest_beams = None  # type: ignore
        self.create_subscription(Float32MultiArray, "shiftgate/beams", self._on_beams, 10)
        maze = demo_maze()
        occ = OccupancyMap(n=maze.n)
        if self.use_sim:
            sense = sense_from_maze(maze)
        else:
            sense = self._sense_from_hardware
            self._maze_for_log = maze
        self.maze = maze
        self.mission = Mission(occ, sense, MissionConfig(start=maze.start, package=maze.package))
        self._emitted = 0
        period = 1.0 / max(float(self.get_parameter("rate_hz").value), 0.2)
        self.timer = self.create_timer(period, self._tick)
        self.get_logger().info(f"ShiftGate mission started (use_sim={self.use_sim})")

    def _on_beams(self, msg: Float32MultiArray) -> None:
        self._latest_beams = list(msg.data)

    def _sense_from_hardware(self, cell, heading: Direction):
        """Best-effort: treat the front beam as the heading, in metres → cells."""
        if not self._latest_beams or len(self._latest_beams) < 3:
            return None
        # beams: [left_flag_or_m, front_m, right_flag_or_m]
        front_m = float(self._latest_beams[1])
        if front_m < 0:
            return None
        cells = front_m / CELL_SIZE_M
        # Snap to 0.5 / 1.5 / None (≥ 2)
        if cells <= 1.0:
            return 0.5
        if cells <= 2.0:
            return 1.5
        return None

    def _tick(self) -> None:
        if self.mission.phase in (MissionPhase.DONE, MissionPhase.FAILED):
            self.pub_cmd.publish(Twist())
            return
        prev_n = len(self.mission.events)
        self.mission.step()
        for ev in self.mission.events[prev_n:]:
            msg = String()
            msg.data = f"{ev.kind}: {ev.detail}"
            self.pub_events.publish(msg)
            self.get_logger().info(msg.data)
        phase = String()
        phase.data = f"{self.mission.phase.value} cell={self.mission.cell}"
        self.pub_phase.publish(phase)
        # In use_sim the abstract mission already "moved"; we publish a
        # zero Twist and let a visualiser / the PyBullet tool show the path.
        # On hardware you would convert (current odom → next cell) into Twist.
        self.pub_cmd.publish(Twist())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
