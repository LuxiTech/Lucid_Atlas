import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__("luxi_set_initial_pose")
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)


def parse_args(argv):
    parser = argparse.ArgumentParser(description="Publish one /initialpose message.")
    parser.add_argument("x", type=float, help="Initial x in the current RViz map frame.")
    parser.add_argument("y", type=float, help="Initial y in the current RViz map frame.")
    parser.add_argument("yaw_deg", type=float, nargs="?", default=0.0, help="Initial yaw in degrees.")
    parser.add_argument("--frame-id", default="map")
    parser.add_argument("--z", type=float, default=0.0)
    return parser.parse_args(argv)


def main(args=None):
    parsed = parse_args(sys.argv[1:] if args is None else args)
    rclpy.init()
    node = InitialPosePublisher()
    try:
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = parsed.frame_id
        msg.pose.pose.position.x = parsed.x
        msg.pose.pose.position.y = parsed.y
        msg.pose.pose.position.z = parsed.z
        yaw = math.radians(parsed.yaw_deg)
        msg.pose.pose.orientation.z = math.sin(yaw * 0.5)
        msg.pose.pose.orientation.w = math.cos(yaw * 0.5)

        # Give discovery a brief moment so the one-shot message is not missed.
        time.sleep(0.5)
        node.publisher.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.5)
        node.get_logger().info(
            "published /initialpose x=%.3f y=%.3f yaw=%.1fdeg"
            % (parsed.x, parsed.y, parsed.yaw_deg)
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
