# GO2 实物适配导航方案（从当前仿真工程迁移）

本文面向当前工程（`go2_robot2/src/projects/mapping_lab`），回答以下问题：

- 是否需要新建工程文件夹
- 实物控制建议走哪条链路（高层/低层）
- 集成实机链路如何在 RViz 展示实物数据
- 如何把狗的数据接入当前系统并复用 Nav2/SLAM
- 项目结构如何安排，便于后续持续扩展

---

## 0. 仓库边界说明（重要）

- `/root/ws/unitree_ros2` 是你拉取的官方仓库根目录。
- 本文提到的 `go2_robot`、`go2_robot2`、`unitree-go2-ros2` 等，是该官方仓库工作区中集成/并存的子工程。
- 因此本文中的“实机入口”是“当前工作区可用入口”，不代表官方仓库唯一标准路径。

---

## 1. 先给结论

1. 建议新建“实物专用”工程层，不要把仿真和实物强耦合在同一 launch。
2. 导航场景建议优先走当前集成驱动的高层运动接口（`cmd_vel -> /api/sport/request`），不要直接用低层关节控制做 Nav2。
3. 当前集成链路的 RViz 可视化本质是：`robot_state_publisher + joint_states + odom/tf + lidar点云/scan`。
4. 你当前系统已经具备大部分导航层（Nav2 + 参数 + RViz），缺的是“稳定实物数据接入层”和“实机安全治理层”。

---

## 2. 现有代码里和实物相关的“集成入口”

你当前工作区里已经有一套可直接参考的实物链路（`go2_robot`）：

- 子工程总入口 launch：  
[go2.launch.py](/root/ws/unitree_ros2/luxitech/go2/go2_robot/go2_bringup/launch/go2.launch.py)
- 驱动 launch：  
[go2_driver.launch.py](/root/ws/unitree_ros2/luxitech/go2/go2_robot/go2_driver/launch/go2_driver.launch.py)
- 驱动核心节点：  
[go2_driver.cpp](/root/ws/unitree_ros2/luxitech/go2/go2_robot/go2_driver/src/go2_driver/go2_driver.cpp)
- 子工程 RViz 配置：  
[go2_rviz.rviz](/root/ws/unitree_ros2/luxitech/go2/go2_robot/go2_rviz/config/go2_rviz.rviz)
- 机器人模型发布：  
[robot.launch.py](/root/ws/unitree_ros2/luxitech/go2/go2_robot/go2_description/launch/robot.launch.py)

从代码可见，`go2_driver` 已经在做：

- 订阅 `cmd_vel`，转换成 `unitree_api::msg::Request` 发到 `/api/sport/request`（高层运动）
- 发布 `/joint_states`、`/odom`、`/pointcloud`，并广播 `odom -> base_link` TF
- 将 `/utlidar/cloud` 通过 `pointcloud_to_laserscan` 转成可用于导航的激光输入链路

这就是你迁移实机最关键的“桥接层”。

---

## 3. 为什么导航建议优先用高层接口

### 3.1 高层接口（推荐）

链路：

`Nav2(cmd_vel) -> go2_driver -> /api/sport/request -> 实机运动`

优点：

- 与 Nav2 输入输出语义天然一致（速度控制）
- 能复用当前集成链路中的运动安全约束和模式切换
- 调参与问题定位更集中在导航层

### 3.2 低层关节控制（不建议作为导航主通道）

低层适合：

- 自定义步态
- 力控/关节级实验

不适合直接承接 Nav2：

- 导航层期望“底盘速度模型”，不是关节级闭环
- 工程复杂度和风险显著增大

---

## 4. 当前集成链路是怎么在 RViz 看实物信息的

本质四件事：

1. 发布机器人模型 `/robot_description`（`robot_state_publisher`）
2. 发布关节状态 `/joint_states`（驱动从 lowstate 映射）
3. 发布位姿和 TF（`/odom` + `odom->base_link`）
4. 发布传感器（`/pointcloud` 或 `/scan`）

对应工程实现：

- 模型：`go2_description/launch/robot.launch.py`
- 驱动：`go2_driver/go2_driver.cpp`
- RViz：`go2_rviz/config/go2_rviz.rviz`（显示 RobotModel + TF + PointCloud2）

---

## 5. 如何把实机数据接到你当前 mapping_lab 系统

你的 `mapping_lab` 主要消费以下标准话题/TF：

- `/cmd_vel`
- `/scan`
- `/odom`
- `/joint_states`
- TF：`map -> odom -> base_link -> lidar_frame`

所以迁移关键是“话题契约一致”。

建议接入顺序：

1. 先跑当前集成实机链路，确认能在 RViz 看到模型和点云
2. 对齐 frame 命名（例如 `radar/front_laser`）
3. 确认 `/scan` 可用（必要时从点云转换）
4. 再挂 Nav2（先手动 teleop，后自主导航）

---

## 6. 建议的新项目结构（实机与仿真解耦）

建议在 `projects/` 下新增实机专用工程层（建议，不强制）：

```text
src/projects/
├── mapping_lab/                         # 现有仿真工程（保留）
└── go2_field_nav/                       # 新增：实机导航工程
    ├── go2_field_bringup/               # 实机启动编排
    │   ├── launch/
    │   │   ├── hw_sensors.launch.py
    │   │   ├── hw_localization.launch.py
    │   │   ├── hw_nav2.launch.py
    │   │   └── hw_full.launch.py
    │   └── scripts/
    │       ├── safety_gate.py
    │       └── cmd_vel_watchdog.py
    ├── go2_field_config/                # 实机参数
    │   ├── config/
    │   │   ├── nav2/nav2_hw_params.yaml
    │   │   ├── amcl/amcl_hw.yaml
    │   │   └── slam_toolbox/mapper_hw.yaml
    │   ├── rviz/
    │   └── maps/
    └── go2_field_msgs/                  # 可选：项目自定义消息/服务
```

说明：

- `mapping_lab` 继续负责仿真迭代
- `go2_field_nav` 负责实物部署
- 两边共享一套“接口约定”，但参数分离

---

## 7. 实机导航落地分阶段（建议路线）

## Phase A：链路打通（不启导航）

目标：只验证“看得到 + 控得动”

1. 网络和 DDS 打通（参考根目录 `setup.sh` 思路）
2. 启动 `go2_bringup/go2.launch.py`（必要时带 lidar/realsense）
3. RViz 检查：TF、RobotModel、PointCloud/Scan、Odom
4. 手动 `cmd_vel` 小速度测试

通过标准：

- `/odom` 连续
- `/scan` 频率稳定
- `/joint_states` 正常
- 机器人响应 `cmd_vel`

## Phase B：定位（Localization）

目标：先把定位稳定下来

1. 先上 AMCL + 已知地图（不用 SLAM）
2. 先人工 `2D Pose Estimate`，再全局重定位
3. 让机器人原地小范围运动，观察 `amcl_pose` 收敛

通过标准：

- `map->odom` 稳定
- 机器人在 RViz 中位置不漂移

## Phase C：导航（Nav2）

目标：自主到点

1. 接 `planner_server + controller_server + behavior_server`
2. 配置 `nav2_hw_params.yaml`（不要直接复用仿真参数）
3. 逐步提速：先慢速验证安全，再提高速度

通过标准：

- 可重复完成 2D Goal 到点
- 窄通道不频繁卡死
- 失败恢复可触发（spin/back_up/clear costmap）

## Phase D：场景化增强

- 动态障碍
- 失联/急停策略
- 自动充电/任务调度（后续）

---

## 8. 实机参数和仿真参数为什么必须分开

仿真与实机差异：

- 传感器噪声、时延、丢包
- 地面摩擦、打滑、速度响应非线性
- 里程计精度和漂移模型不同

因此建议：

- 新建 `nav2_hw_params.yaml`
- 从保守参数起步（更低速度、更大安全边界）
- 在实机环境按风险等级逐步调

---

## 9. 安全与控制建议（强烈建议）

1. 增加 `safety_gate` 节点：
- 限速（线速度/角速度上限）
- 区域限行（禁入区域）
- 状态门控（跌倒、低电量、遥控抢占时禁止导航输出）

2. 增加 `cmd_vel_watchdog`：
- 若导航节点超时无更新，自动输出 0 速度

3. 保留人工接管优先级：
- 遥控器/急停优先级高于 Nav2

4. 分层启动：
- 先传感器，再定位，再导航，最后任务层

---

## 10. 与当前 mapping_lab 的对接方式

建议维护两套 launch：

1. `sim_nav2.launch.py`（现有）
- Gazebo 仿真 + Nav2

2. `hw_nav2.launch.py`（新增）
- go2_driver + robot_state_publisher + lidar驱动 + Nav2

两者共用：

- 目标话题语义（`/cmd_vel`, `/scan`, `/odom`）
- RViz 操作方式（2D Pose Estimate / 2D Goal Pose）

---

## 11. 常见问题（FAQ）

1. 一定要新建工程文件夹吗？  
不是必须，但强烈建议。实机安全策略、参数、启动逻辑与仿真差异很大，分层后维护成本最低。

2. 可以直接用当前 `nav2_sim_params.yaml` 吗？  
不建议。可作为初始模板，但必须复制出 `nav2_hw_params.yaml` 单独调。

3. 为何 RViz 能看到模型但不能导航？  
通常是 TF 链或里程计链不闭合（`map->odom->base_link`）、或 `scan` 帧与 costmap 配置不匹配。

4. 数据怎么“传到当前系统”？  
通过 DDS/ROS2 话题直接进来。关键是网络接口、DDS 配置、话题名和 frame 对齐。

5. 是否建议直接用高层运动控制接口（当前集成链路）？  
是。对于导航控制，优先高层接口最稳、最安全、集成成本最低。

---

## 12. 最小可行实机链路（MVP）

1. 启动：`go2_description + go2_driver + lidar`  
2. 确认：`/scan /odom /joint_states /tf`  
3. 接入：`nav2_bringup + hw_params + map_server + amcl`  
4. 验证：RViz 下发 2D Goal，低速通过窄门/转角  
5. 上线前：加入安全门控与 watchdog

---

## 13. 推荐下一步（可执行）

1. 在 `projects` 下创建 `go2_field_nav`（实机包骨架）。
2. 从 `sim_nav2.launch.py` 复制一份 `hw_nav2.launch.py`，移除 Gazebo，接 `go2_bringup`。
3. 新建 `nav2_hw_params.yaml`（从保守参数开始）。
4. 增加 `safety_gate.py` 与 `cmd_vel_watchdog.py`。
5. 完成一次“实机低速到点”验收。

---

## 14. 实机接入命令清单（建议按顺序执行）

> 下面是“先数据、后定位、再导航”的最小流程示例。

### 14.1 基础环境

```bash
source /opt/ros/humble/setup.bash
source /root/ws/unitree_ros2/setup.sh
source /root/ws/unitree_ros2/luxitech/go2/go2_robot2/install/setup.bash
```

### 14.2 启动当前集成实机链路（含驱动）

```bash
ros2 launch go2_bringup go2.launch.py lidar:=True rviz:=True
```

### 14.3 验证“狗的数据”是否进入 ROS2

```bash
ros2 topic list | rg 'odom|joint_states|pointcloud|scan|lowstate|api/sport/request'
ros2 topic hz /odom
ros2 topic hz /joint_states
ros2 topic hz /scan
ros2 topic echo /odom --once
ros2 topic echo /joint_states --once
```

### 14.4 验证 TF 链

```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link radar
```

如果某一段 TF 不通，先修 frame 命名，不要急着调 Nav2。

### 14.5 小速度控制闭环验证（安全场地）

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 5
```

停止：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -1
```

### 14.6 再接入 Nav2（建议先 AMCL + 地图）

实机 `hw_nav2.launch.py` 准备好后，流程与仿真一致：

- `2D Pose Estimate` 给初始位姿
- `2D Goal Pose` 下发目标点
- 观察 `global plan / local plan / costmap`

---

## 15. 实机迁移最常见“卡点”与处理优先级

1. `scan` 有但导航不动：  
先查 `TF`（通常是 `scan frame` 没挂到 `base_link` 链上）。

2. RViz 能看见模型但 AMCL 漂：  
先查 `/odom` 连续性与时间戳，再看激光时间同步。

3. `cmd_vel` 发了狗不动：  
先查 `go2_driver` 是否在收 `cmd_vel`，再查 `/api/sport/request` 是否持续发布，最后查机器狗当前运动模式是否允许受控。

4. 实机经常“贴墙”或“保守不走”：  
不要先改全局规划，先改 `nav2_hw_params.yaml` 中局部代价地图与 DWB 参数。

优先级建议始终是：

`网络/DDS -> 话题 -> TF -> 里程计 -> 定位 -> 局部控制器参数`
