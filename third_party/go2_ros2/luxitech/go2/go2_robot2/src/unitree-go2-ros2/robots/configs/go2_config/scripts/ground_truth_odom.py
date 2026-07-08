#!/usr/bin/env python3

import copy
import math
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class GroundTruthOdomBridge(Node):
    def __init__(self):
        super().__init__("ground_truth_odom_bridge")
        self._odom_publisher = self.create_publisher(Odometry, "odom", 10)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._subscription = self.create_subscription(
            Odometry,
            "odom/ground_truth",
            self._handle_ground_truth,
            10,
        )
        self._initial_position: Optional[Tuple[float, float, float]] = None
        self._initial_orientation: Optional[Tuple[float, float, float, float]] = None

    @staticmethod
    def _quat_inverse(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
        x, y, z, w = q
        norm = x * x + y * y + z * z + w * w
        if norm <= 0.0:
            return (0.0, 0.0, 0.0, 1.0)
        return (-x / norm, -y / norm, -z / norm, w / norm)

    @staticmethod
    def _quat_multiply(
        q1: Tuple[float, float, float, float],
        q2: Tuple[float, float, float, float],
    ) -> Tuple[float, float, float, float]:
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        return (
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        )

    @classmethod
    def _rotate_vector(
        cls,
        q: Tuple[float, float, float, float],
        vector: Tuple[float, float, float],
    ) -> Tuple[float, float, float]:
        vector_quat = (vector[0], vector[1], vector[2], 0.0)
        rotated = cls._quat_multiply(
            cls._quat_multiply(q, vector_quat),
            cls._quat_inverse(q),
        )
        return (rotated[0], rotated[1], rotated[2])

    @staticmethod
    def _normalize_quaternion(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
        x, y, z, w = q
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm <= 0.0:
            return (0.0, 0.0, 0.0, 1.0)
        return (x / norm, y / norm, z / norm, w / norm)

    @staticmethod
    def _to_tuple(quat: Quaternion) -> Tuple[float, float, float, float]:
        return (quat.x, quat.y, quat.z, quat.w)

    @staticmethod
    def _yaw_from_quaternion(q: Tuple[float, float, float, float]) -> float:
        x, y, z, w = q
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
        half_yaw = 0.5 * yaw
        return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))

    def _handle_ground_truth(self, msg: Odometry) -> None:
        odom_msg = copy.deepcopy(msg)
        current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        )
        current_orientation = self._normalize_quaternion(self._to_tuple(msg.pose.pose.orientation))

        if self._initial_position is None or self._initial_orientation is None:
            self._initial_position = current_position
            self._initial_orientation = current_orientation

        relative_position = (
            current_position[0] - self._initial_position[0],
            current_position[1] - self._initial_position[1],
            current_position[2] - self._initial_position[2],
        )
        initial_inverse = self._quat_inverse(self._initial_orientation)
        local_position = self._rotate_vector(initial_inverse, relative_position)
        local_orientation = self._normalize_quaternion(
            self._quat_multiply(initial_inverse, current_orientation)
        )
        local_yaw = self._yaw_from_quaternion(local_orientation)
        planar_orientation = self._quaternion_from_yaw(local_yaw)

        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = local_position[0]
        odom_msg.pose.pose.position.y = local_position[1]
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = planar_orientation[0]
        odom_msg.pose.pose.orientation.y = planar_orientation[1]
        odom_msg.pose.pose.orientation.z = planar_orientation[2]
        odom_msg.pose.pose.orientation.w = planar_orientation[3]
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        self._odom_publisher.publish(odom_msg)

        transform = TransformStamped()
        transform.header = odom_msg.header
        transform.child_frame_id = odom_msg.child_frame_id
        transform.transform.translation.x = odom_msg.pose.pose.position.x
        transform.transform.translation.y = odom_msg.pose.pose.position.y
        transform.transform.translation.z = odom_msg.pose.pose.position.z
        transform.transform.rotation = odom_msg.pose.pose.orientation
        self._tf_broadcaster.sendTransform(transform)


def main() -> None:
    rclpy.init()
    node = GroundTruthOdomBridge()
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
