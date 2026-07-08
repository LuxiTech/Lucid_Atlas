# mapping_lab_bringup 使用说明（中文）

本文档用于说明：

- 可选地图有哪些
- 如何切换地图
- 如何用键盘控制 GO2
- 常用指令合集（可直接复制）

## 1. 环境准备

每次新终端先执行：

```bash
source /opt/ros/humble/setup.bash
source /root/ws/unitree_ros2/luxitech/go2/go2_robot2/install/setup.bash
```

## 2. 可选地图清单

当前可用 world 文件位于：

`/root/ws/unitree_ros2/luxitech/go2/go2_robot2/src/projects/mapping_lab/mapping_lab_config/worlds`

可选地图：

- `turtlebot3_house_classic.world`（默认）
- `turtlebot3_world_classic.world`
- `turtlebot3_dqn_classic.world`
- `lidar_lab.world`

## 3. 启动仿真与 RViz

默认启动（Gazebo + RViz）：

```bash
ros2 launch mapping_lab_bringup sim_lidar_view.launch.py gui:=true
```

说明：

- 当前默认地图是 `turtlebot3_house_classic.world`
- 默认固定出生点：`world_init_x:=1.5 world_init_y:=-2.0 world_init_z:=0.35 world_init_heading:=0.0`
- 默认不暂停物理：`paused:=false`（更利于控制器和 joint_states 正常激活）

## 4. 切换地图

通过 `world:=<world绝对路径>` 切换。

示例 1：切到 house 地图

```bash
ros2 launch mapping_lab_bringup sim_lidar_view.launch.py gui:=true \
  world:=/root/ws/unitree_ros2/luxitech/go2/go2_robot2/src/projects/mapping_lab/mapping_lab_config/worlds/turtlebot3_house_classic.world \
  world_init_x:=1.5 world_init_y:=-2.0 world_init_z:=0.35 world_init_heading:=0.0 \
  paused:=false
```

示例 2：切到 dqn 地图

```bash
ros2 launch mapping_lab_bringup sim_lidar_view.launch.py gui:=true \
  world:=/root/ws/unitree_ros2/luxitech/go2/go2_robot2/src/projects/mapping_lab/mapping_lab_config/worlds/turtlebot3_dqn_classic.world \
  world_init_x:=0.0 world_init_y:=0.0 world_init_z:=0.35 world_init_heading:=0.0 \
  paused:=false
```

示例 3：切到自定义 lidar_lab 地图

```bash
ros2 launch mapping_lab_bringup sim_lidar_view.launch.py gui:=true \
  world:=/root/ws/unitree_ros2/luxitech/go2/go2_robot2/src/projects/mapping_lab/mapping_lab_config/worlds/lidar_lab.world
```

## 5. 键盘控制 GO2

新开一个终端（先 source 环境），运行：

```bash
ros2 run champ_teleop champ_teleop.py
```

常用按键（以终端提示为准）：

- `w` 前进
- `s` 后退
- `a` 左移
- `d` 右移
- `q` 左转
- `e` 右转
- `space` 或 `k` 急停
- `r/f` 增减线速度
- `t/g` 增减角速度

## 6. 常用排障指令

1. 先清理残留进程再重启：

```bash
pkill -f 'sim_lidar_view.launch.py|sim_nav2.launch.py|sim_slam_toolbox.launch.py|gzserver|gzclient|rviz2|robot_state_publisher|quadruped_controller_node|spawn_entity.py|ground_truth_odom.py|initial_pose_publisher.py|global_localization_trigger.py|amcl|planner_server|controller_server|bt_navigator|lifecycle_manager|map_server|smoother_server|behavior_server|waypoint_follower|velocity_smoother' || true
```

2. 检查雷达是否发布：

```bash
ros2 topic hz /scan
```

3. 检查关节状态是否正常：

```bash
ros2 topic info /joint_states -v
```

4. 检查时钟（仿真时间）：

```bash
ros2 topic info /clock -v
```

5. 机器人“瞬移/飞来飞去”时先检查里程计发布源（`/odom` 发布者应为 1）：

```bash
ros2 topic info /odom -v
```

## 7. 一套最常用的完整流程

终端 1：

```bash
source /opt/ros/humble/setup.bash
source /root/ws/unitree_ros2/luxitech/go2/go2_robot2/install/setup.bash
pkill -f 'sim_lidar_view.launch.py|sim_nav2.launch.py|sim_slam_toolbox.launch.py|gzserver|gzclient|rviz2|robot_state_publisher|quadruped_controller_node|spawn_entity.py|ground_truth_odom.py|initial_pose_publisher.py|global_localization_trigger.py|amcl|planner_server|controller_server|bt_navigator|lifecycle_manager|map_server|smoother_server|behavior_server|waypoint_follower|velocity_smoother' || true
ros2 launch mapping_lab_bringup sim_lidar_view.launch.py gui:=true
```

终端 2：

```bash
source /opt/ros/humble/setup.bash
source /root/ws/unitree_ros2/luxitech/go2/go2_robot2/install/setup.bash
ros2 run champ_teleop champ_teleop.py
```

## 8. 使用 slam_toolbox 建图

### 8.1 代码位置

- `slam_toolbox` 源码：  
`/root/ws/unitree_ros2/luxitech/go2/go2_robot2/src/third_party/slam_toolbox`
- 建图参数：  
`/root/ws/unitree_ros2/luxitech/go2/go2_robot2/src/projects/mapping_lab/mapping_lab_config/config/slam_toolbox/mapper_params_online_async.yaml`
- 建图启动文件：  
`/root/ws/unitree_ros2/luxitech/go2/go2_robot2/src/projects/mapping_lab/mapping_lab_bringup/launch/sim_slam_toolbox.launch.py`

### 8.2 编译

```bash
source /opt/ros/humble/setup.bash
cd /root/ws/unitree_ros2/luxitech/go2/go2_robot2
colcon build --packages-select slam_toolbox mapping_lab_config mapping_lab_bringup
source install/setup.bash
```

### 8.3 启动建图

终端 1（启动仿真 + slam_toolbox + RViz）：

```bash
source /opt/ros/humble/setup.bash
source /root/ws/unitree_ros2/luxitech/go2/go2_robot2/install/setup.bash
pkill -f 'sim_lidar_view.launch.py|sim_nav2.launch.py|sim_slam_toolbox.launch.py|gzserver|gzclient|rviz2|robot_state_publisher|quadruped_controller_node|spawn_entity.py|ground_truth_odom.py|initial_pose_publisher.py|global_localization_trigger.py|amcl|planner_server|controller_server|bt_navigator|lifecycle_manager|map_server|smoother_server|behavior_server|waypoint_follower|velocity_smoother' || true
ros2 launch mapping_lab_bringup sim_slam_toolbox.launch.py gui:=true
```

终端 2（键盘控制）：

```bash
source /opt/ros/humble/setup.bash
source /root/ws/unitree_ros2/luxitech/go2/go2_robot2/install/setup.bash
ros2 run champ_teleop champ_teleop.py
```

### 8.4 保存地图

保存为 `pgm+yaml`（例如保存到 `maps/lab_map`）：

```bash
source /opt/ros/humble/setup.bash
source /root/ws/unitree_ros2/luxitech/go2/go2_robot2/install/setup.bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/root/ws/unitree_ros2/luxitech/go2/go2_robot2/maps/lab_map'}}"
```

可选：保存 pose graph（后续继续建图可用）：

```bash
source /opt/ros/humble/setup.bash
source /root/ws/unitree_ros2/luxitech/go2/go2_robot2/install/setup.bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/root/ws/unitree_ros2/luxitech/go2/go2_robot2/maps/lab_map.posegraph'}"
```

## 9. 使用 Nav2 做定位与导航（基于已保存地图）

说明：

- `sim_lidar_view.launch.py` 只负责仿真与雷达显示，不包含导航。
- `sim_nav2.launch.py` 会启动：仿真 + Nav2（map_server/amcl/planner/controller）+ RViz。
- 当前默认地图为：  
`/root/ws/unitree_ros2/luxitech/go2/go2_robot2/src/projects/mapping_lab/mapping_lab_config/maps/lab_map.yaml`

### 9.1 编译导航相关包

```bash
source /opt/ros/humble/setup.bash
cd /root/ws/unitree_ros2/luxitech/go2/go2_robot2
colcon build --packages-select mapping_lab_config mapping_lab_bringup
source install/setup.bash
```

### 9.2 启动导航

```bash
source /opt/ros/humble/setup.bash
source /root/ws/unitree_ros2/luxitech/go2/go2_robot2/install/setup.bash
pkill -f 'sim_lidar_view.launch.py|sim_nav2.launch.py|sim_slam_toolbox.launch.py|gzserver|gzclient|rviz2|robot_state_publisher|quadruped_controller_node|spawn_entity.py|ground_truth_odom.py|initial_pose_publisher.py|global_localization_trigger.py|amcl|planner_server|controller_server|bt_navigator|lifecycle_manager|map_server|smoother_server|behavior_server|waypoint_follower|velocity_smoother' || true
ros2 launch mapping_lab_bringup sim_nav2.launch.py gui:=true nav2_rviz:=true
```

### 9.3 导航操作顺序

1. 默认会自动发布一次初始位姿（`auto_initial_pose:=true`，默认 `x=1.5,y=-2.0,yaw=0.0`）。
2. 如果你希望手动粗定位，再在 RViz 使用 `2D Pose Estimate` 覆盖自动初始位姿即可。
3. 再点击 `2D Goal Pose` 下发目标点，机器人会自动规划并运动（并显示全局/局部路径）。
4. 不要同时开键盘 teleop 和 Nav2，避免 `/cmd_vel` 冲突。

说明：

- 默认 Nav2 目标点是在 RViz 下发，不是 Gazebo 场景内直接点击。
- Gazebo 主要用于物理仿真展示，导航交互入口建议使用 RViz。

### 9.4 仿真位置和地图不对应时的处理

如果每次启动都偏移明显，说明当前地图与启动位姿基准不一致。可以直接传入初始位姿参数：

```bash
ros2 launch mapping_lab_bringup sim_nav2.launch.py gui:=true nav2_rviz:=true \
  world_init_x:=1.5 world_init_y:=-2.0 world_init_z:=0.35 world_init_heading:=0.0 \
  auto_initial_pose:=true initial_pose_x:=1.5 initial_pose_y:=-2.0 initial_pose_yaw:=0.0 \
  paused:=false
```

建议调参顺序：

1. 先调 `initial_pose_x` / `initial_pose_y` 让机器人平移对齐。
2. 再调 `initial_pose_yaw`（弧度）做角度对齐。
3. 对齐后把参数写回你常用启动命令。

如果偏差非常大（例如房间结构整体都对不上），建议重新在当前 world 和固定出生点下建图并保存新地图。

### 9.5 实物场景（起点不固定）推荐模式

AMCL 不是“仅靠一帧雷达就绝对定位”的系统。地图对称、视野有限、动态障碍都会导致多解，因此通常需要：

1. 全局重定位（把粒子撒满地图）
2. 机器人转动/短距离运动，让 AMCL 逐步收敛

启动示例（不使用固定初始位姿，改为全局重定位）：

```bash
ros2 launch mapping_lab_bringup sim_nav2.launch.py gui:=true nav2_rviz:=true \
  auto_initial_pose:=false use_global_localization:=true
```

然后让机器人原地缓慢转一圈（或小范围移动），待 `amcl_pose` 稳定后再下发导航目标。

### 9.6 如果 Gazebo / RViz 窗口没有弹出（X11 权限）

若日志出现 `Authorization required`、`could not connect to display`、`xcb` 等报错，请先授权 X11：

1. 在桌面用户终端执行（非 root）：

```bash
xhost +SI:localuser:root
```

2. 在 root 终端执行：

```bash
export DISPLAY=:1
export XAUTHORITY=/home/<你的桌面用户名>/.Xauthority
```

示例：`export XAUTHORITY=/home/fxx/.Xauthority`

### 9.7 切换地图导航

```bash
ros2 launch mapping_lab_bringup sim_nav2.launch.py gui:=true \
  map:=/root/ws/unitree_ros2/luxitech/go2/go2_robot2/maps/lab_map.yaml
```

## 10. Nav2 调参思路（现象驱动）

本项目导航主参数文件：

`/root/ws/unitree_ros2/luxitech/go2/go2_robot2/src/projects/mapping_lab/mapping_lab_config/config/nav2/nav2_sim_params.yaml`

建议调参方法（非常重要）：

1. 一次只改一类参数（最多 2~3 个），避免互相干扰。
2. 每次小步调整，观察 3~5 次不同目标点结果再决定下一步。
3. 改完都要重启 launch（Nav2 参数不是全部支持在线热更新）。

### 10.1 常见现象 -> 该调哪些参数

1. 现象：靠墙后“被吸住”、不再前进，或在墙角僵住。  
优先调：
- `FollowPath.BaseObstacle.scale`（避障权重）  
调大：`+0.03 ~ +0.08`（更不敢贴墙）
- `local_costmap/global_costmap.robot_radius`（机器人膨胀半径）  
调大：`+0.01 ~ +0.02`（安全边界更大）
- `local/global inflation_layer.inflation_radius`（危险区范围）  
调大：`+0.03 ~ +0.08`
- `local/global inflation_layer.cost_scaling_factor`（代价下降斜率）  
调大：更“陡”，靠墙代价上升更快；调小：更平滑但可能贴墙。

2. 现象：到门口反复前进/后退，路线来回重规划。  
优先调：
- `FollowPath.Oscillation.*`（振荡抑制）
- `FollowPath.PathDist.scale`、`FollowPath.PathAlign.scale`（贴全局路径）
- `FollowPath.sim_time`（局部轨迹预测时域）
经验：
- 振荡明显：先调大 `Oscillation.scale`（每次 `+0.2 ~ +0.8`）
- 门口来回绕：调大 `PathDist.scale`（每次 `+2 ~ +6`）
- 反应太激进：适当减小 `sim_time`（如 `2.0 -> 1.6`）

3. 现象：不能原地掉头，必须绕很大弯。  
优先调：
- `FollowPath.max_vel_theta`
- `FollowPath.min_speed_theta`（建议允许 `0.0`）
- `FollowPath.RotateToGoal.scale`（提高原地转目标朝向倾向）
- `velocity_smoother.max_velocity[2]` 与 `min_velocity[2]`（必须同步）
经验：
- 原地转不积极：`RotateToGoal.scale` 每次 `+8 ~ +20`
- 转速不够：`max_vel_theta` 每次 `+0.3 ~ +0.8`

4. 现象：明明有空隙却过不去（过于保守）。  
优先调：
- `robot_radius` 适度减小（每次 `-0.01`）
- `inflation_radius` 适度减小（每次 `-0.02 ~ -0.05`）
- `BaseObstacle.scale` 略降（每次 `-0.02 ~ -0.05`）
注意：这类调整会降低安全冗余，务必小步并多次验证。

5. 现象：轨迹抖动、左右横跳、速度忽快忽慢。  
优先调：
- `FollowPath.vx_samples/vy_samples/vtheta_samples`（采样密度）
- `FollowPath.short_circuit_trajectory_evaluation`（通常建议 `false`）
- `velocity_smoother.max_accel/max_decel`
经验：
- 抖动大：适当降低最大速度，先保稳定再提速。

### 10.2 先检查再调参（排除“假问题”）

1. 检查是否多节点重复发布里程计（会导致“瞬移/飞来飞去”）：

```bash
ros2 topic info /odom -v
```

`/odom` 发布者建议为 1。

2. 检查雷达数据是否稳定：

```bash
ros2 topic hz /scan
ros2 topic echo /scan --once
```

3. 检查控制器实时参数是否已生效：

```bash
ros2 param get /controller_server FollowPath.BaseObstacle.scale
ros2 param get /controller_server FollowPath.max_vel_theta
ros2 param get /controller_server FollowPath.RotateToGoal.scale
```

### 10.3 推荐调参顺序（从稳到快）

1. 先保证定位稳定（AMCL 不漂移、不瞬移）。
2. 再调安全边界（`robot_radius + inflation_radius`）。
3. 再调局部规划行为（`BaseObstacle / PathDist / Oscillation / RotateToGoal`）。
4. 最后再提速（`max_vel_x/y/theta` 与 `velocity_smoother` 同步）。

### 10.4 每次改完后的固定流程

```bash
cd /root/ws/unitree_ros2/luxitech/go2/go2_robot2
source /opt/ros/humble/setup.bash
colcon build --packages-select mapping_lab_config mapping_lab_bringup
source install/setup.bash
ros2 launch mapping_lab_bringup sim_nav2.launch.py gui:=true nav2_rviz:=true
```
