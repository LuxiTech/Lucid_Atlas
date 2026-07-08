# GO2 cmd_vel Bridge

This ROS 2 C++ package subscribes to `geometry_msgs/msg/Twist` on `/cmd_vel`
and publishes Unitree Go2 Sport API requests on `/api/sport/request`.

- Non-zero `linear.x`, `linear.y`, or `angular.z` commands publish `Move`
  (`api_id=1008`) with `{"x": vx, "y": vy, "z": yaw_rate}`.
- Zero commands publish `StopMove` (`api_id=1003`).
- If commands stop arriving after the first command, the watchdog publishes one
  `StopMove` after `cmd_timeout_sec`.
- Velocity limits default to `max_linear_x=0.5`, `max_linear_y=0.5`,
  `max_angular_z=1.0`.

ROS 2 subscription callbacks do not expose the publisher source IP address, so
the `expected_sender_ip` parameter is informational. Restricting traffic to
`192.168.123.51` should be handled with the DDS network configuration, firewall,
or ROS domain/network setup.

## Build

```bash
source /opt/ros/humble/setup.bash
cd /home/nvidia/project/luxi-atlas/third_party/go2_ros2/cyclonedds_ws
colcon build --symlink-install --packages-select unitree_api unitree_go unitree_hg
source install/setup.bash

cd /home/nvidia/project/luxi-atlas/third_party/go2_ros2/luxitech/go2/go2_robot2
colcon build --symlink-install --packages-select go2_cmd_vel_bridge
```

## Run

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/project/luxi-atlas/third_party/go2_ros2/cyclonedds_ws/install/setup.bash
source install/setup.bash
ros2 launch go2_cmd_vel_bridge go2_cmd_vel_bridge.launch.py
```

## Local Test

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/project/luxi-atlas/third_party/go2_ros2/cyclonedds_ws/install/setup.bash
source install/setup.bash
ros2 run go2_cmd_vel_bridge go2_cmd_vel_bridge_node
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.25, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}"
ros2 topic echo --once /api/sport/request
```
