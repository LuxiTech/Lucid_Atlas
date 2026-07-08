#!/usr/bin/env python3

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


class SafetyGate(Node):
    def __init__(self) -> None:
        super().__init__("safety_gate")
        self.declare_parameter("input_topic", "/cmd_vel_raw")
        self.declare_parameter("output_topic", "/cmd_vel")
        self.declare_parameter("max_linear_x", 0.5)
        self.declare_parameter("max_linear_y", 0.3)
        self.declare_parameter("max_angular_z", 1.5)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self._max_x = abs(float(self.get_parameter("max_linear_x").value))
        self._max_y = abs(float(self.get_parameter("max_linear_y").value))
        self._max_w = abs(float(self.get_parameter("max_angular_z").value))

        self._pub = self.create_publisher(Twist, output_topic, 10)
        self._sub = self.create_subscription(Twist, input_topic, self._on_cmd, 10)
        self.get_logger().info(
            f"Safety gate enabled: {input_topic} -> {output_topic} "
            f"(vx<= {self._max_x:.2f}, vy<= {self._max_y:.2f}, wz<= {self._max_w:.2f})"
        )

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        return max(-limit, min(limit, value))

    def _on_cmd(self, msg: Twist) -> None:
        safe = Twist()
        safe.linear.x = self._clamp(msg.linear.x, self._max_x)
        safe.linear.y = self._clamp(msg.linear.y, self._max_y)
        safe.angular.z = self._clamp(msg.angular.z, self._max_w)
        self._pub.publish(safe)


def main() -> None:
    rclpy.init()
    node = SafetyGate()
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

