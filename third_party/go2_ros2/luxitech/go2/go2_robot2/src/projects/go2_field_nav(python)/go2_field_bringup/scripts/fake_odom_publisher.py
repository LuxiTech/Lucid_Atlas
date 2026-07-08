#!/usr/bin/env python3

import math

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class FakeOdomPublisher(Node):
    def __init__(self) -> None:
        super().__init__("fake_odom_publisher")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("child_frame_id", "base_link")
        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("z", 0.0)
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("publish_rate_hz", 30.0)

        self._frame_id = str(self.get_parameter("frame_id").value)
        self._child_frame_id = str(self.get_parameter("child_frame_id").value)
        self._x = float(self.get_parameter("x").value)
        self._y = float(self.get_parameter("y").value)
        self._z = float(self.get_parameter("z").value)
        self._yaw = float(self.get_parameter("yaw").value)
        self._publish_rate = float(self.get_parameter("publish_rate_hz").value)

        if self._publish_rate <= 0.0:
            self._publish_rate = 30.0

        self._tf_broadcaster = TransformBroadcaster(self)
        self._odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.create_timer(1.0 / self._publish_rate, self._publish_once)
        self.get_logger().warn(
            "fake_odom_publisher enabled. This is for no-hardware bringup validation only."
        )

    def _publish_once(self) -> None:
        now = self.get_clock().now().to_msg()
        half_yaw = 0.5 * self._yaw
        qz = math.sin(half_yaw)
        qw = math.cos(half_yaw)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = now
        tf_msg.header.frame_id = self._frame_id
        tf_msg.child_frame_id = self._child_frame_id
        tf_msg.transform.translation.x = self._x
        tf_msg.transform.translation.y = self._y
        tf_msg.transform.translation.z = self._z
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(tf_msg)

        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = self._frame_id
        odom_msg.child_frame_id = self._child_frame_id
        odom_msg.pose.pose.position.x = self._x
        odom_msg.pose.pose.position.y = self._y
        odom_msg.pose.pose.position.z = self._z
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw
        self._odom_pub.publish(odom_msg)


def main() -> None:
    rclpy.init()
    node = FakeOdomPublisher()
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
