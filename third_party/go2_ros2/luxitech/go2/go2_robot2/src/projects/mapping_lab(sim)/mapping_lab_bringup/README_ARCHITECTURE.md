# mapping_lab_bringup 架构说明（中文）

本文档说明当前工程的系统架构：

- 有哪些核心包与配置
- 三种启动模式分别起了哪些节点
- 为什么同一套模型能实现不同效果（纯仿真 / 建图 / 导航）
- 避障、规划、定位、建图算法分别在哪里调用
- 关键文件之间的依赖关系

---

## 1. 总体目录关系

当前项目主目录：

`/root/ws/unitree_ros2/luxitech/go2/go2_robot2/src/projects/mapping_lab`

核心结构：

```text
mapping_lab/
├── mapping_lab_bringup/                 # 启动编排层（launch + 辅助脚本）
│   ├── launch/
│   │   ├── sim_lidar_view.launch.py     # 仿真+激光+可选RViz
│   │   ├── sim_slam_toolbox.launch.py   # 仿真 + SLAM Toolbox
│   │   └── sim_nav2.launch.py           # 仿真 + Nav2定位导航
│   └── scripts/
│       ├── initial_pose_publisher.py    # 自动发 /initialpose
│       └── global_localization_trigger.py # 调AMCL全局重定位服务
└── mapping_lab_config/                  # 资源配置层
    ├── config/
    │   ├── nav2/nav2_sim_params.yaml    # Nav2参数（控制器/代价地图/AMCL等）
    │   ├── slam_toolbox/mapper_params_online_async.yaml
    │   └── project_topics.yaml
    ├── maps/                            # 已保存地图（lab_map.yaml/.pgm）
    ├── rviz/                            # 各模式RViz配置
    └── worlds/                          # Gazebo world 文件
```

---

## 2. 依赖层次（谁调用谁）

`mapping_lab_bringup` 本身不实现底层控制算法，而是“编排器”：

1. 调用 `champ_bringup/bringup.launch.py`（机器人控制与状态发布）
2. 调用 `champ_gazebo/gazebo.launch.py`（Gazebo、spawn、ros2_control 控制器）
3. 叠加业务节点：
- `ground_truth_odom.py`（里程计桥接）
- `slam_toolbox`（建图）
- `nav2_bringup`（定位+导航）
- RViz（不同视图配置）

因此是“同一仿真底座 + 不同上层节点组合 = 不同效果”。

---

## 3. 三种模式的节点组合

## 3.1 `sim_lidar_view.launch.py`（仿真底座）

用途：只做机器人仿真、雷达显示、手动控制验证。

主要节点来源：

1. 来自 `champ_description/description.launch.py`
- `robot_state_publisher`

2. 来自 `champ_bringup/bringup.launch.py`
- `quadruped_controller_node`（核心步态/关节控制）
- `state_estimation_node`、`ekf_node`（当前在此项目里默认关闭）

3. 来自 `champ_gazebo/gazebo.launch.py`
- `gzserver` / `gzclient`
- `spawn_entity.py`
- `spawner joint_states_controller`
- `spawner joint_group_effort_controller`
- （可选）`contact_sensor`

4. 来自本项目
- `ground_truth_odom_bridge`（`go2_config/scripts/ground_truth_odom.py`）
- `rviz2`（可开关）

结果：有物理仿真、有 `/scan`、有 `/odom`、可键盘控制运动，但不做地图定位与自主导航。

---

## 3.2 `sim_slam_toolbox.launch.py`（仿真 + 建图）

它先 include `sim_lidar_view.launch.py`（并关闭其 RViz），再加：

- `slam_toolbox` (`async_slam_toolbox_node`)
- 建图专用 RViz（`slam_toolbox_view.rviz`）

结果：在仿真底座上增加“在线建图”，输出/维护 `map`，可保存地图。

算法入口文件：

- `mapping_lab_config/config/slam_toolbox/mapper_params_online_async.yaml`

关键参数（示例）：

- `scan_topic: /scan`
- `odom_frame: odom`
- `map_frame: map`
- `base_frame: base_link`

---

## 3.3 `sim_nav2.launch.py`（仿真 + 定位 + 导航）

它先 include `sim_lidar_view.launch.py`（关闭其 RViz），再 include `nav2_bringup/bringup_launch.py`。

额外节点：

1. Nav2 定位链路（`localization_launch.py`）
- `map_server`
- `amcl`
- `lifecycle_manager_localization`

2. Nav2 导航链路（`navigation_launch.py`）
- `planner_server`
- `controller_server`
- `smoother_server`
- `behavior_server`
- `bt_navigator`
- `waypoint_follower`
- `velocity_smoother`
- `lifecycle_manager_navigation`

3. 本项目辅助节点
- `initial_pose_publisher.py`（自动发初始位姿）
- `global_localization_trigger.py`（可选全局重定位）
- `rviz2`（`nav2_view.rviz`）

结果：可在 RViz 下发 `2D Goal Pose`，机器人自主规划与避障到达目标。

---

## 4. 关键算法在哪里“生效”

## 4.1 避障（Obstacle Avoidance）

避障不是单独一个节点，而是由以下模块共同实现：

1. `local_costmap` + `global_costmap`
- 层：`ObstacleLayer` / `VoxelLayer` / `InflationLayer`
- 参数文件：`mapping_lab_config/config/nav2/nav2_sim_params.yaml`

2. `controller_server` 的局部规划器
- `FollowPath.plugin: dwb_core::DWBLocalPlanner`
- DWB critics（如 `BaseObstacle`, `Oscillation`, `PathDist`, `RotateToGoal`）决定“贴墙程度、是否原地转、是否容易振荡”。

所以“被墙吸住、门口来回”本质是 DWB 评分 + costmap 参数耦合结果。

## 4.2 全局路径规划

- 节点：`planner_server`
- 插件：`nav2_navfn_planner/NavfnPlanner`
- `use_astar: true` 表示采用 A* 样式全局搜索。

## 4.3 定位（Localization）

- 节点：`amcl`
- 模型：`nav2_amcl::OmniMotionModel`（支持全向运动模型）
- 输入：`/scan` + `/map` + `/odom`
- 输出：`/amcl_pose` + `map->odom` TF

## 4.4 建图（SLAM）

- 节点：`slam_toolbox`
- 模式：`mode: mapping`
- 输出：实时构建占据栅格地图（`/map`）

## 4.5 机器人运动控制链路

典型路径：

1. 导航模式：
- `controller_server` 先输出 `cmd_vel_nav`
- `velocity_smoother` 平滑后重映射到 `cmd_vel`
- `quadruped_controller_node` 订阅 `cmd_vel`
- 发布关节轨迹到 `joint_group_effort_controller/joint_trajectory`
- Gazebo ros2_control 执行关节力矩/运动

2. 键盘模式：
- `champ_teleop.py` 直接发 `cmd_vel`
- 后续链路相同

---

## 5. 传感器与模型关系（为什么能看到激光）

GO2 使用模型：

- `luxi_go2_description/xacro/go2_l1_sim.xacro`

该模型 include：

- `l1_front_laser.xacro`

雷达插件在 `l1_front_laser.xacro` 中定义：

- Gazebo 传感器类型：`ray`
- ROS 插件：`libgazebo_ros_ray_sensor.so`
- 话题重映射：`~/out := scan`，最终发布 `/scan`
- 帧名：`front_laser`

因此，RViz 里显示 `LaserScan` 时 topic 选 `/scan` 即可。

---

## 6. TF 与里程计结构（仿真里为什么还要桥接）

本项目使用 `ground_truth_odom_bridge`（`go2_config/scripts/ground_truth_odom.py`）：

1. 订阅 `odom/ground_truth`
2. 以首次位置为原点做重基准
3. 发布标准 `/odom`
4. 同时广播 `odom -> base_link` TF

目的：

- 让导航链路使用稳定一致的 odom 原点
- 减少仿真初始偏移对定位和导航的干扰

注意：

- 若误起多个同类节点，会出现多个 `/odom` 发布者，导致“瞬移/飞来飞去”。

---

## 7. 为什么“同一工程，不同 launch 效果不同”

本质是“节点组合不同”：

1. `sim_lidar_view`：只有仿真底座 + 传感 + 控制
2. `sim_slam_toolbox`：在 1 基础上加在线建图
3. `sim_nav2`：在 1 基础上加地图定位、全局规划、局部避障与行为树决策

因此不是“换了机器人模型”，而是“上层智能模块是否加载”的区别。

---

## 8. 关键文件索引（快速定位）

启动编排：

- `mapping_lab_bringup/launch/sim_lidar_view.launch.py`
- `mapping_lab_bringup/launch/sim_slam_toolbox.launch.py`
- `mapping_lab_bringup/launch/sim_nav2.launch.py`

导航与建图参数：

- `mapping_lab_config/config/nav2/nav2_sim_params.yaml`
- `mapping_lab_config/config/slam_toolbox/mapper_params_online_async.yaml`

辅助脚本：

- `mapping_lab_bringup/scripts/initial_pose_publisher.py`
- `mapping_lab_bringup/scripts/global_localization_trigger.py`
- `unitree-go2-ros2/robots/configs/go2_config/scripts/ground_truth_odom.py`

模型与雷达：

- `luxi_go2_description/xacro/go2_l1_sim.xacro`
- `luxi_go2_description/xacro/l1_front_laser.xacro`

---

## 9. 调试建议（架构视角）

1. 先确认节点层级是否符合模式预期（仿真/建图/导航）。
2. 再确认 TF 链条：`map -> odom -> base_link -> front_laser`。
3. 再看话题来源是否唯一（尤其 `/odom`、`/cmd_vel`）。
4. 最后再调算法参数（DWB critics、costmap inflation、AMCL 粒子等）。

推荐命令：

```bash
ros2 node list
ros2 topic info /odom -v
ros2 topic hz /scan
ros2 run tf2_tools view_frames
```

