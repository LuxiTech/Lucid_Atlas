#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class PointCloudRelay(Node):
    def __init__(self) -> None:
        super().__init__("pointcloud_relay")
        self.declare_parameter("input_topic", "/utlidar/cloud")
        self.declare_parameter("output_topic", "/pointcloud")
        self.declare_parameter("force_frame_id", "radar")

        self._input_topic = str(self.get_parameter("input_topic").value)
        self._output_topic = str(self.get_parameter("output_topic").value)
        self._force_frame_id = str(self.get_parameter("force_frame_id").value)

        self._pub = self.create_publisher(PointCloud2, self._output_topic, 10)
        self.create_subscription(PointCloud2, self._input_topic, self._on_cloud, 10)
        self.get_logger().info(
            f"PointCloud relay started: {self._input_topic} -> {self._output_topic}"
        )

    def _on_cloud(self, msg: PointCloud2) -> None:
        if self._force_frame_id:
            msg.header.frame_id = self._force_frame_id
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = PointCloudRelay()
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
