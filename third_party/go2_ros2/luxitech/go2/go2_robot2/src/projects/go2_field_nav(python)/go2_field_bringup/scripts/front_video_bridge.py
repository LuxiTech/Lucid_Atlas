#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from unitree_go.msg import Go2FrontVideoData


class FrontVideoBridge(Node):
    def __init__(self) -> None:
        super().__init__("front_video_bridge")
        self.declare_parameter("input_topic", "/front_video_data")
        self.declare_parameter("output_topic", "/front_camera/image_raw")
        self.declare_parameter("frame_id", "front_camera")
        self.declare_parameter("log_warn_interval_sec", 2.0)

        self._input_topic = str(self.get_parameter("input_topic").value)
        self._output_topic = str(self.get_parameter("output_topic").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._warn_interval = float(self.get_parameter("log_warn_interval_sec").value)
        self._last_warn = self.get_clock().now()
        self._logged_stream_info = False

        self._pub = self.create_publisher(Image, self._output_topic, 10)
        self.create_subscription(Go2FrontVideoData, self._input_topic, self._on_msg, 10)
        self.get_logger().info(
            f"Front video bridge started: {self._input_topic} -> {self._output_topic}"
        )

    def _warn_throttled(self, text: str) -> None:
        now = self.get_clock().now()
        dt = (now - self._last_warn).nanoseconds * 1e-9
        if dt >= self._warn_interval:
            self._last_warn = now
            self.get_logger().warn(text)

    def _on_msg(self, msg: Go2FrontVideoData) -> None:
        if not msg.data:
            self._warn_throttled("front video payload is empty")
            return

        if not self._logged_stream_info:
            payload = bytes(msg.data)
            signature = payload[:8].hex()
            self.get_logger().info(
                "front video stream detected: "
                f"resolution={msg.resolution}, payload_size={len(payload)}, "
                f"signature=0x{signature}"
            )
            self._logged_stream_info = True

        arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if frame is None:
            self._warn_throttled(
                "failed to decode Go2FrontVideoData with JPEG decoder; "
                "payload may be non-JPEG (e.g. H264/H265)."
            )
            return

        image_msg = Image()
        image_msg.header.stamp = self.get_clock().now().to_msg()
        image_msg.header.frame_id = self._frame_id
        image_msg.height = int(frame.shape[0])
        image_msg.width = int(frame.shape[1])
        image_msg.encoding = "bgr8"
        image_msg.is_bigendian = 0
        image_msg.step = int(frame.shape[1] * frame.shape[2])
        image_msg.data = frame.tobytes()
        self._pub.publish(image_msg)


def main() -> None:
    rclpy.init()
    node = FrontVideoBridge()
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
