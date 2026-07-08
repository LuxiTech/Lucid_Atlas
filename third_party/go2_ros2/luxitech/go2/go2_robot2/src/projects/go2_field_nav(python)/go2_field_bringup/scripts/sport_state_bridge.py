#!/usr/bin/env python3

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from unitree_go.msg import SportModeState


class SportStateBridge(Node):
    def __init__(self) -> None:
        super().__init__("sport_state_bridge")
        self.declare_parameter("sport_state_topic", "/lf/sportmodestate")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("position_z_offset", 0.07)
        self.declare_parameter("quaternion_order", "wxyz")
        self.declare_parameter("publish_tf", True)

        self._sport_state_topic = str(self.get_parameter("sport_state_topic").value)
        self._odom_topic = str(self.get_parameter("odom_topic").value)
        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._z_offset = float(self.get_parameter("position_z_offset").value)
        self._quaternion_order = str(self.get_parameter("quaternion_order").value).lower()
        self._publish_tf = bool(self.get_parameter("publish_tf").value)

        self._odom_pub = self.create_publisher(Odometry, self._odom_topic, 10)
        self._tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            SportModeState, self._sport_state_topic, self._on_state, 10
        )

        self.get_logger().info(
            "Sport state bridge started: "
            f"{self._sport_state_topic} -> {self._odom_topic}, "
            f"tf {self._odom_frame}->{self._base_frame}, "
            f"quaternion_order={self._quaternion_order}"
        )

    def _parse_quaternion(self, quat) -> tuple[float, float, float, float]:
        if self._quaternion_order == "xyzw":
            return float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])
        # Unitree convention is generally wxyz; keep as default.
        return float(quat[1]), float(quat[2]), float(quat[3]), float(quat[0])

    def _on_state(self, msg: SportModeState) -> None:
        stamp = self.get_clock().now().to_msg()
        qx, qy, qz, qw = self._parse_quaternion(msg.imu_state.quaternion)
        px = float(msg.position[0])
        py = float(msg.position[1])
        pz = float(msg.position[2]) + self._z_offset

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = px
        odom.pose.pose.position.y = py
        odom.pose.pose.position.z = pz
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = float(msg.velocity[0])
        odom.twist.twist.linear.y = float(msg.velocity[1])
        odom.twist.twist.linear.z = float(msg.velocity[2])
        odom.twist.twist.angular.z = float(msg.yaw_speed)
        self._odom_pub.publish(odom)

        if self._publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = self._odom_frame
            tf_msg.child_frame_id = self._base_frame
            tf_msg.transform.translation.x = px
            tf_msg.transform.translation.y = py
            tf_msg.transform.translation.z = pz
            tf_msg.transform.rotation.x = qx
            tf_msg.transform.rotation.y = qy
            tf_msg.transform.rotation.z = qz
            tf_msg.transform.rotation.w = qw
            self._tf_broadcaster.sendTransform(tf_msg)


def main() -> None:
    rclpy.init()
    node = SportStateBridge()
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
