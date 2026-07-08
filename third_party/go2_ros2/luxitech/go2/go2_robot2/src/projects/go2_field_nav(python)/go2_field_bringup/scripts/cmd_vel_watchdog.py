#!/usr/bin/env python3

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


class CmdVelWatchdog(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_watchdog")
        self.declare_parameter("topic", "/cmd_vel")
        self.declare_parameter("timeout_sec", 0.7)
        self.declare_parameter("warn_interval_sec", 1.5)

        self._topic = self.get_parameter("topic").value
        self._timeout = float(self.get_parameter("timeout_sec").value)
        self._warn_interval = float(self.get_parameter("warn_interval_sec").value)
        self._last_cmd_time = self.get_clock().now()
        self._last_warn_time = self.get_clock().now()
        self._first_received = False

        self.create_subscription(Twist, self._topic, self._on_cmd, 10)
        self.create_timer(0.1, self._on_timer)
        self.get_logger().info(
            f"Watchdog started on {self._topic} (timeout={self._timeout:.2f}s)"
        )

    def _on_cmd(self, _msg: Twist) -> None:
        self._last_cmd_time = self.get_clock().now()
        self._first_received = True

    def _on_timer(self) -> None:
        if not self._first_received:
            return
        now = self.get_clock().now()
        dt = (now - self._last_cmd_time).nanoseconds * 1e-9
        dw = (now - self._last_warn_time).nanoseconds * 1e-9
        if dt > self._timeout and dw > self._warn_interval:
            self._last_warn_time = now
            self.get_logger().warn(
                f"No cmd_vel on {self._topic} for {dt:.2f}s (timeout={self._timeout:.2f}s)"
            )


def main() -> None:
    rclpy.init()
    node = CmdVelWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

