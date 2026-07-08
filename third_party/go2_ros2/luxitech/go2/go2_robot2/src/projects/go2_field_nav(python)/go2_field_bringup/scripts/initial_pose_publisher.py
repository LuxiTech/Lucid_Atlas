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

        self._frame_id = self.get_parameter("frame_id").value
        self._x = float(self.get_parameter("x").value)
        self._y = float(self.get_parameter("y").value)
        self._yaw = float(self.get_parameter("yaw").value)
        self._cov_xy = float(self.get_parameter("covariance_xy").value)
        self._cov_yaw = float(self.get_parameter("covariance_yaw").value)
        self._repeat_count = int(self.get_parameter("repeat_count").value)
        self._publish_period = float(self.get_parameter("publish_period_sec").value)

        self._pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self._published = 0
        self._timer = self.create_timer(self._publish_period, self._publish_once)

    def _publish_once(self) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.pose.pose.position.x = self._x
        msg.pose.pose.position.y = self._y
        msg.pose.pose.position.z = 0.0

        half_yaw = 0.5 * self._yaw
        msg.pose.pose.orientation.z = math.sin(half_yaw)
        msg.pose.pose.orientation.w = math.cos(half_yaw)
        msg.pose.covariance[0] = self._cov_xy
        msg.pose.covariance[7] = self._cov_xy
        msg.pose.covariance[35] = self._cov_yaw

        self._pub.publish(msg)
        self._published += 1
        if self._published >= self._repeat_count:
            self._timer.cancel()
            self.get_logger().info("Initial pose publishing completed.")


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

