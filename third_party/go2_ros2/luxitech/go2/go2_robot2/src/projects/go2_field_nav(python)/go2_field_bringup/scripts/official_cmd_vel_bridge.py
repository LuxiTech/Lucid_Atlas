#!/usr/bin/env python3

import json

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from unitree_api.msg import Request


class OfficialCmdVelBridge(Node):
    def __init__(self) -> None:
        super().__init__("official_cmd_vel_bridge")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("request_topic", "/api/sport/request")
        self.declare_parameter("move_api_id", 1008)
        self.declare_parameter("max_linear_x", 0.8)
        self.declare_parameter("max_linear_y", 0.6)
        self.declare_parameter("max_angular_z", 1.8)

        self._cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self._request_topic = str(self.get_parameter("request_topic").value)
        self._move_api_id = int(self.get_parameter("move_api_id").value)
        self._max_linear_x = float(self.get_parameter("max_linear_x").value)
        self._max_linear_y = float(self.get_parameter("max_linear_y").value)
        self._max_angular_z = float(self.get_parameter("max_angular_z").value)
        self._request_id = 1

        self._pub = self.create_publisher(Request, self._request_topic, 10)
        self.create_subscription(Twist, self._cmd_vel_topic, self._on_cmd_vel, 10)
        self.get_logger().info(
            "Official cmd_vel bridge started: "
            f"{self._cmd_vel_topic} -> {self._request_topic} (api_id={self._move_api_id})"
        )

    @staticmethod
    def _clamp(value: float, bound: float) -> float:
        if value > bound:
            return bound
        if value < -bound:
            return -bound
        return value

    def _on_cmd_vel(self, msg: Twist) -> None:
        x = self._clamp(float(msg.linear.x), self._max_linear_x)
        y = self._clamp(float(msg.linear.y), self._max_linear_y)
        z = self._clamp(float(msg.angular.z), self._max_angular_z)

        req = Request()
        req.header.identity.id = self._request_id
        req.header.identity.api_id = self._move_api_id
        req.parameter = json.dumps({"x": x, "y": y, "z": z})
        self._pub.publish(req)
        self._request_id += 1


def main() -> None:
    rclpy.init()
    node = OfficialCmdVelBridge()
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
