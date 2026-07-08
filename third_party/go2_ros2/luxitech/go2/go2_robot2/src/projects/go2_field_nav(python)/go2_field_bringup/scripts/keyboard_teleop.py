#!/usr/bin/env python3

import select
import sys
import termios
import tty
from typing import BinaryIO, Optional

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


class KeyboardTeleop(Node):
    def __init__(self) -> None:
        super().__init__("keyboard_teleop")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("linear_speed", 0.6)
        self.declare_parameter("lateral_speed", 0.6)
        self.declare_parameter("angular_speed", 1.0)
        self.declare_parameter("key_timeout_sec", 0.25)
        self.declare_parameter("publish_hz", 20.0)

        self._cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self._linear_speed = float(self.get_parameter("linear_speed").value)
        self._lateral_speed = float(self.get_parameter("lateral_speed").value)
        self._angular_speed = float(self.get_parameter("angular_speed").value)
        self._key_timeout = float(self.get_parameter("key_timeout_sec").value)
        publish_hz = float(self.get_parameter("publish_hz").value)
        self._timer_period = 1.0 / publish_hz if publish_hz > 0.0 else 0.05

        self._pub = self.create_publisher(Twist, self._cmd_vel_topic, 10)
        self._target = Twist()
        self._last_key_time = self.get_clock().now()
        self._key_stream: Optional[BinaryIO] = None
        self._stdin_fd = self._resolve_input_fd()
        self._old_tty_attr = termios.tcgetattr(self._stdin_fd)

        tty.setraw(self._stdin_fd)
        self.create_timer(self._timer_period, self._on_timer)

        self.get_logger().info(
            "Keyboard teleop started. "
            f"Publishing to {self._cmd_vel_topic}. "
            "W/S forward/back, A/D left/right, Left/Right arrow turn, Space/K stop, Ctrl-C quit."
        )

    def _resolve_input_fd(self) -> int:
        # Prefer controlling terminal so this node can still work under ros2 launch.
        try:
            self._key_stream = open("/dev/tty", "rb", buffering=0)
            return self._key_stream.fileno()
        except OSError:
            pass

        if sys.stdin.isatty():
            return sys.stdin.fileno()

        raise RuntimeError(
            "No interactive TTY detected for keyboard input. "
            "Please launch from a terminal or disable enable_keyboard_teleop."
        )

    def destroy_node(self) -> bool:
        try:
            termios.tcsetattr(self._stdin_fd, termios.TCSADRAIN, self._old_tty_attr)
        except Exception:
            pass
        try:
            if self._key_stream is not None:
                self._key_stream.close()
        except Exception:
            pass
        return super().destroy_node()

    def _set_target(self, x: float, y: float, z: float) -> None:
        self._target.linear.x = x
        self._target.linear.y = y
        self._target.angular.z = z
        self._last_key_time = self.get_clock().now()

    def _stop(self) -> None:
        self._target = Twist()
        self._last_key_time = self.get_clock().now()

    def _read_key(self) -> str:
        if not select.select([self._stdin_fd], [], [], 0.0)[0]:
            return ""
        c1 = self._read_char()
        if c1 != "\x1b":
            return c1
        if not select.select([self._stdin_fd], [], [], 0.0)[0]:
            return c1
        c2 = self._read_char()
        if c2 != "[":
            return c1 + c2
        if not select.select([self._stdin_fd], [], [], 0.0)[0]:
            return c1 + c2
        c3 = self._read_char()
        return c1 + c2 + c3

    def _read_char(self) -> str:
        ch = b""
        if self._key_stream is not None:
            ch = self._key_stream.read(1) or b""
        else:
            ch = sys.stdin.buffer.read(1) or b""
        return ch.decode("latin-1")

    def _handle_key(self, key: str) -> None:
        if not key:
            return
        if key == "\x03":
            self.get_logger().info("Keyboard teleop received Ctrl-C, shutting down.")
            self._stop()
            self._pub.publish(self._target)
            raise SystemExit
        elif key == "w":
            self._set_target(self._linear_speed, 0.0, 0.0)
        elif key == "s":
            self._set_target(-self._linear_speed, 0.0, 0.0)
        elif key == "a":
            self._set_target(0.0, self._lateral_speed, 0.0)
        elif key == "d":
            self._set_target(0.0, -self._lateral_speed, 0.0)
        elif key in ("q", "\x1b[D"):
            self._set_target(0.0, 0.0, self._angular_speed)
        elif key in ("e", "\x1b[C"):
            self._set_target(0.0, 0.0, -self._angular_speed)
        elif key in (" ", "k", "x"):
            self._stop()

    def _on_timer(self) -> None:
        key = self._read_key()
        self._handle_key(key)

        dt = (self.get_clock().now() - self._last_key_time).nanoseconds * 1e-9
        if dt > self._key_timeout:
            self._target = Twist()

        self._pub.publish(self._target)


def main() -> None:
    rclpy.init()
    node = KeyboardTeleop()
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
