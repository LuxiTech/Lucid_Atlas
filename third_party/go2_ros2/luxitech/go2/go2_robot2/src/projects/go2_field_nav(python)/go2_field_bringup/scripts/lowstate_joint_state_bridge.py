#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from unitree_go.msg import LowState


class LowStateJointStateBridge(Node):
    def __init__(self) -> None:
        super().__init__("lowstate_joint_state_bridge")
        self.declare_parameter("lowstate_topic", "/lowstate")
        self.declare_parameter("joint_states_topic", "/joint_states")

        self._lowstate_topic = str(self.get_parameter("lowstate_topic").value)
        self._joint_states_topic = str(self.get_parameter("joint_states_topic").value)

        self._joint_names = [
            "FL_hip_joint",
            "FL_thigh_joint",
            "FL_calf_joint",
            "FR_hip_joint",
            "FR_thigh_joint",
            "FR_calf_joint",
            "RL_hip_joint",
            "RL_thigh_joint",
            "RL_calf_joint",
            "RR_hip_joint",
            "RR_thigh_joint",
            "RR_calf_joint",
        ]

        # Match GO2 motor index mapping used by validated driver implementations.
        self._motor_idx = [3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8]

        self._pub = self.create_publisher(JointState, self._joint_states_topic, 10)
        self.create_subscription(LowState, self._lowstate_topic, self._on_lowstate, 10)

        self.get_logger().info(
            "LowState->JointState bridge started: "
            f"{self._lowstate_topic} -> {self._joint_states_topic}"
        )

    def _on_lowstate(self, msg: LowState) -> None:
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self._joint_names
        js.position = [float(msg.motor_state[i].q) for i in self._motor_idx]
        self._pub.publish(js)


def main() -> None:
    rclpy.init()
    node = LowStateJointStateBridge()
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
