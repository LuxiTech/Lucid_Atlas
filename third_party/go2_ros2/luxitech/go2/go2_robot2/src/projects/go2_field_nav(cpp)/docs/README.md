# go2_field_nav(cpp) 二次开发说明

本目录已替换为 `unitree-go2-slam-toolbox` 结构，并完成与当前仓库的使用适配。

目标：
- 在 RViz 查看 GO2 实机模型与传感数据
- 启动 2D 建图（slam_toolbox）
- 保存地图并启动 Nav2 导航

## 0. 防冲突运行原则（重要）

为避免“机器人静止但 RViz 位姿持续漂移/跳动”，请严格遵循：

1. 一个流程只运行一套栈
- 建图时只启动 `go2_start.launch.py`
- 导航时只启动 `go2_nav2.launch.py`
- 不要同时运行 demo 工程和本工程（除非使用不同 `ROS_DOMAIN_ID`）

2. ROS Domain 必须与机器人一致
- GO2 实机常见为 `ROS_DOMAIN_ID=0`（以现场为准）
- 所有终端必须使用同一个 `ROS_DOMAIN_ID`
- 若设成其他值（如 66），会出现“只有模型、无状态/无雷达/无地图”

3. 启动前清理旧进程
- 上一次异常退出后，旧节点可能残留并重复发布 `/odom` 或 TF

## 1. 工程结构

源码根目录：

`/root/ws/unitree_ros2/luxitech/go2/go2_robot2/src/projects/go2_field_nav(cpp)/src`

包含包：

- `base/go2_core`：主入口，串联 driver + localization + perception + slam + RViz
- `base/go2_driver`：状态桥接、TF、IMU、关节状态
- `base/go2_twist_bridge`：`/cmd_vel -> /api/sport/request`
- `go2_perception`：点云累积与 `PointCloud2 -> LaserScan`
- `go2_slam`：slam_toolbox 启动与参数
- `go2_navigation2`：Nav2 启动与参数（已适配默认地图路径）
- `go2_description`：机器人模型描述

## 2. 数据链路实现

核心链路如下：

1. 运动与状态（官方接口）
- 订阅：`/lf/sportmodestate`、`/lf/lowstate`、`/utlidar/robot_pose`
- 控制发布：`/api/sport/request`（高层运动接口）

2. 感知
- 输入：`/utlidar/cloud_deskewed`
- 点云累积：`/utlidar/cloud_accumulated`
- 激光扫描：`/scan`

3. 建图
- `slam_toolbox` 订阅 `/scan`
- 产出 `/map` 与 `map -> odom` TF

4. 导航
- Nav2 读取保存地图 yaml
- 使用 AMCL + costmap + planner/controller 执行导航

## 3. 编译方式（推荐，隔离构建目录）

在 `go2_robot2` 根目录执行：

```bash
source /root/ws/unitree_ros2/setup.sh
source /opt/ros/humble/setup.bash
cd /root/ws/unitree_ros2/luxitech/go2/go2_robot2
export ROS_DOMAIN_ID=0

colcon --log-base "src/projects/go2_field_nav(cpp)/log" build --symlink-install \
  --base-paths "src/projects/go2_field_nav(cpp)/src" \
  --build-base "src/projects/go2_field_nav(cpp)/build" \
  --install-base "src/projects/go2_field_nav(cpp)/install"
```

运行前加载环境：

```bash
source /root/ws/unitree_ros2/setup.sh
source /opt/ros/humble/setup.bash
source "/root/ws/unitree_ros2/luxitech/go2/go2_robot2/src/projects/go2_field_nav(cpp)/install/setup.bash"
export ROS_DOMAIN_ID=0
```

启动前建议先执行（清理残留节点）：

```bash
ros2 daemon stop || true
pkill -f "go2_|slam_toolbox|nav2_|amcl|map_server|ekf_node|robot_state_publisher|pointcloud_to_laserscan|cloud_accumulation|twist_bridge|footprint_to_link|rviz2" || true
sleep 2
ros2 daemon start
```

## 4. RViz 查看 GO2（模型 + 状态）

```bash
ros2 launch go2_driver driver.launch.py use_rviz:=true
```

可快速检查：

```bash
ros2 topic hz /joint_states
ros2 topic hz /odom
ros2 topic hz /scan
```

## 5. 启动建图（SLAM）

终端 A（先清理后启动）：

```bash
source /root/ws/unitree_ros2/setup.sh
source /opt/ros/humble/setup.bash
source "/root/ws/unitree_ros2/luxitech/go2/go2_robot2/src/projects/go2_field_nav(cpp)/install/setup.bash"
export ROS_DOMAIN_ID=0

ros2 daemon stop || true
pkill -f "go2_|slam_toolbox|nav2_|amcl|map_server|ekf_node|robot_state_publisher|pointcloud_to_laserscan|cloud_accumulation|twist_bridge|footprint_to_link|rviz2" || true
sleep 2
ros2 daemon start

ros2 launch go2_core go2_start.launch.py
```

终端 B（遥控建图）：

```bash
source /root/ws/unitree_ros2/setup.sh
source /opt/ros/humble/setup.bash
source "/root/ws/unitree_ros2/luxitech/go2/go2_robot2/src/projects/go2_field_nav(cpp)/install/setup.bash"
export ROS_DOMAIN_ID=0

ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

说明：
- `go2_start.launch.py` 默认启用 `go2_slam/go2_slamtoolbox.launch.py`
- RViz 使用 `go2_core/rviz2/display.rviz`，可直接看到 `/map`

建图自检（另开终端）：

```bash
ros2 topic info /odom
ros2 topic info /scan
ros2 node list | grep -E "slam_toolbox|driver|ekf_filter_node|cloud_accumulation|pointcloud_to_laserscan"
```

期望：
- `/odom` 只有 1 个发布者（来自当前驱动链路）
- `/scan` 正常有数据
- 存在 `slam_toolbox` 节点

## 6. 保存地图

建图完成后，另开终端执行：

```bash
ros2 run nav2_map_server map_saver_cli \
  -f "/root/ws/unitree_ros2/luxitech/go2/go2_robot2/src/projects/go2_field_nav(cpp)/src/go2_navigation2/maps/my_map"
```

将生成：
- `my_map.pgm`
- `my_map.yaml`

## 7. 启动导航（Nav2）

本工程已适配 `go2_navigation2/launch/go2_nav2.launch.py`：
- 去除重复 `map_server/amcl` 启动
- 默认地图改为包内 `maps/lab_map.yaml`
- 统一串联 driver + localization + perception + nav2

启动命令：

```bash
source /root/ws/unitree_ros2/setup.sh
source /opt/ros/humble/setup.bash
source "/root/ws/unitree_ros2/luxitech/go2/go2_robot2/src/projects/go2_field_nav(cpp)/install/setup.bash"
export ROS_DOMAIN_ID=0

# 若当前在建图，先 Ctrl+C 停掉 go2_start.launch.py 再启动导航
ros2 launch go2_navigation2 go2_nav2.launch.py
```

使用自定义地图：

```bash
ros2 launch go2_navigation2 go2_nav2.launch.py \
  map:="/root/ws/unitree_ros2/luxitech/go2/go2_robot2/src/projects/go2_field_nav(cpp)/src/go2_navigation2/maps/my_map.yaml"
```

导航操作：
- 在 RViz 使用 `2D Pose Estimate` 设初始位姿
- 使用 `2D Goal Pose` 下发导航目标

## 8. 常见问题

1. 只看到模型看不到地图
- 确认 `slam_toolbox` 节点是否存在：`ros2 node list | grep slam_toolbox`
- 确认 `/scan` 有数据：`ros2 topic hz /scan`

2. 机器人静止但 RViz 位姿持续变化
- 核心原因通常是重复发布（最常见是混跑了 demo 与当前工程，或旧进程残留）
- 重点检查：`ros2 topic info /odom`，确认发布者数量不是 2 个及以上
- 严格执行“启动前清理 + `ROS_DOMAIN_ID` 与机器人一致 + 只跑一套 launch”

3. 启动冲突/包混乱
- 不要在同一个 shell 同时 source 多个不同工程 install
- 优先使用本说明的隔离 `build/install/log` 路径

4. 只看到模型，看不到状态/雷达/地图
- 先检查上游话题发布者是否为 1：
  - `ros2 topic info /lf/lowstate`
  - `ros2 topic info /lf/sportmodestate`
  - `ros2 topic info /utlidar/cloud_deskewed`
- 若发布者为 0，优先检查 `ROS_DOMAIN_ID` 是否与机器人一致（通常恢复为 `0`）

5. 导航失败
- 检查 `map`、`odom`、`base_link/base_footprint` TF 是否连通
- 先确保建图链路稳定，再进行导航参数调优

## 9. 依赖建议

```bash
sudo apt update
sudo apt install -y \
  ros-humble-robot-localization \
  ros-humble-slam-toolbox \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-teleop-twist-keyboard
```

## 10. 对比 demo 的安全方式

若你要对比：
`/root/ws/unitree_ros2/luxitech/go2/go2_robot2/demo/unitree-go2-slam-toolbox`

建议“同一时刻只运行一个工程”，都使用与机器人一致的 Domain（通常 0）：

```bash
export ROS_DOMAIN_ID=0
```

说明：
- demo 与当前工程都需要订阅同一套实机话题，不建议靠改 Domain 同时跑。
- 需要对比时，先完全停止一套，再启动另一套。
