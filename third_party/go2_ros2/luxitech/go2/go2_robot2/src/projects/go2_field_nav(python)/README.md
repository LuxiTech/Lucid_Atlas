# go2_field_nav

实机 GO2 导航工程骨架（与 `mapping_lab` 仿真工程解耦）。

包含：

- `go2_field_bringup`：实机启动编排
- `go2_field_config`：实机参数与配置

默认策略：
- 控制与通信优先使用官方高层接口（`/api/sport/request`, `/lf/sportmodestate`）
- RViz 仅承担可视化职责

## Phase A（链路打通）启动

```bash
source /opt/ros/humble/setup.bash
source /root/ws/unitree_ros2/setup.sh
source /root/ws/unitree_ros2/luxitech/go2/go2_robot/install/setup.bash
source /root/ws/unitree_ros2/luxitech/go2/go2_robot2/install/setup.bash

ros2 launch go2_field_bringup hw_sensors.launch.py rviz:=true
```

说明：`hw_sensors`、`hw_nav2`、`hw_full` 当前都默认 `enable_cmd_vel_bridge:=false`，优先保留官方遥控器控制。
导航阶段需要显式打开高层运动桥接：

```bash
ros2 launch go2_field_bringup hw_nav2.launch.py enable_cmd_vel_bridge:=true
```

如已安装 `pointcloud_to_laserscan`，可开启点云转激光：

```bash
ros2 launch go2_field_bringup hw_sensors.launch.py rviz:=true enable_scan_bridge:=true
```

如果当前没有接实机（仅做本地链路联调），可临时打开假里程计 TF（仅联调用）：

```bash
ros2 launch go2_field_bringup hw_nav2.launch.py \
  rviz:=false \
  enable_scan_bridge:=false \
  enable_fake_odom:=true \
  enable_fake_map_to_odom:=true
```

> 注意：`enable_fake_odom`、`enable_fake_map_to_odom` 只用于“无实机”验证 launch 和 Nav2 生命周期是否能拉起，实机部署必须关闭。

验证：

```bash
ros2 topic hz /odom
ros2 topic hz /joint_states
ros2 topic hz /scan
ros2 topic echo /odom --once
```
