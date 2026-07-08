#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty


class GlobalLocalizationTrigger(Node):
    def __init__(self) -> None:
        super().__init__("global_localization_trigger")
        self.declare_parameter("service_name", "/reinitialize_global_localization")
        self.declare_parameter("wait_timeout_sec", 30.0)
        service_name = self.get_parameter("service_name").value
        wait_timeout = float(self.get_parameter("wait_timeout_sec").value)

        self._client = self.create_client(Empty, service_name)
        self.get_logger().info(f"Waiting for service: {service_name}")
        if not self._client.wait_for_service(timeout_sec=wait_timeout):
            self.get_logger().error(f"Service {service_name} unavailable in {wait_timeout:.1f}s")
            self._shutdown()
            return

        future = self._client.call_async(Empty.Request())
        future.add_done_callback(self._done_cb)

    def _done_cb(self, _future) -> None:
        self.get_logger().info("Global localization service triggered.")
        self._shutdown()

    def _shutdown(self) -> None:
        self.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> None:
    rclpy.init()
    node = GlobalLocalizationTrigger()
    if rclpy.ok():
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            if node.context.ok():
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()

