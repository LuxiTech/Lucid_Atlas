#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


class InitialPosePublisher(Node):
    def __init__(self) -> None:
        super().__init__("initial_pose_publisher")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("covariance_xy", 0.25)
        self.declare_parameter("covariance_yaw", 0.0685)
        self.declare_parameter("repeat_count", 20)
        self.declare_parameter("publish_period_sec", 0.2)

        self._frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self._x = self.get_parameter("x").get_parameter_value().double_value
        self._y = self.get_parameter("y").get_parameter_value().double_value
        self._yaw = self.get_parameter("yaw").get_parameter_value().double_value
        self._cov_xy = self.get_parameter("covariance_xy").get_parameter_value().double_value
        self._cov_yaw = self.get_parameter("covariance_yaw").get_parameter_value().double_value
        self._repeat_count = self.get_parameter("repeat_count").get_parameter_value().integer_value
        self._publish_period = (
            self.get_parameter("publish_period_sec").get_parameter_value().double_value
        )

        self._pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self._published = 0
        self._timer = self.create_timer(self._publish_period, self._publish_once)
        self.get_logger().info(
            f"Publishing initial pose in frame '{self._frame_id}' "
            f"(x={self._x:.3f}, y={self._y:.3f}, yaw={self._yaw:.3f})"
        )

    def _publish_once(self) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.pose.pose.position.x = self._x
        msg.pose.pose.position.y = self._y
        msg.pose.pose.position.z = 0.0

        half_yaw = 0.5 * self._yaw
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(half_yaw)
        msg.pose.pose.orientation.w = math.cos(half_yaw)

        msg.pose.covariance[0] = self._cov_xy
        msg.pose.covariance[7] = self._cov_xy
        msg.pose.covariance[35] = self._cov_yaw

        self._pub.publish(msg)
        self._published += 1

        if self._published >= self._repeat_count:
            self.get_logger().info("Initial pose publishing completed.")
            self._timer.cancel()


def main() -> None:
    rclpy.init()
    node = InitialPosePublisher()
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
