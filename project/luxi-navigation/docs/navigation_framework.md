# Luxi Navigation 第一阶段工程文档

## 1. 阶段目标

第一阶段实现“网页选点 -> 全局路径规划 -> 网页显示路径”。当前已增加 DWB 风格局部规划器，默认只发布 dry-run 速度到 `/nav/cmd_vel`，不直接控制真实底盘。

当前已经实现：

- 订阅 `/map` 静态栅格地图。
- 优先使用 `/localization_2d` 作为机器人地图位姿。
- 调试阶段可使用 `/Odometry` 作为回退位姿。
- 订阅网页目标点 `/web/goal_pose`。
- 使用 2D A* 在膨胀后的栅格地图上规划全局路径。
- 发布 `/nav/path`，网页地图画布绘制导航路线。
- 发布 `/nav/status`，网页和终端都可以看到规划状态。
- 使用 DWB 风格局部轨迹采样器，从全局路径生成局部速度命令。
- 发布 `/nav/cmd_vel`，用于 dry-run 验证。实车闭环前再把 `cmd_vel_topic` 切到 `/cmd_vel` 或接入安全仲裁节点。

## 2. 工程目录

```text
project/luxi-navigation/
  CMakeLists.txt
  package.xml
  config/
    navigation.yaml
  docs/
    navigation_framework.md
  launch/
    navigation.launch.py
  src/
    luxi_navigation_node.cpp
```

网页侧改动位于：

```text
project/luxi-web-control/
  launch/web_navigation.launch.py
  src/web_api_node.cpp
  web/index.html
  web/main.js
  web/styles.css
```

`web_navigation.launch.py` 是推荐的一键启动入口，会按顺序拉起定位、网页服务和导航节点。

## 3. 总体架构

```text
手机浏览器
  |
  | HTTP: http://设备IP:8080
  v
luxi_web_control 静态网页
  |
  | POST /api/goal_pose
  v
web_api_node
  |
  | ROS2 /web/goal_pose
  v
luxi_navigation_node
  |
  | ROS2 /nav/path, /nav/status
  v
web_api_node
  |
  | GET /api/map/snapshot
  v
网页 Canvas 显示地图、定位、目标点、全局路径

luxi_navigation_node
  |
  | ROS2 /nav/cmd_vel
  v
DWB dry-run 速度输出
```

算法侧链路：

```text
/map
  -> 阈值占用判断
  -> 未知区域处理
  -> 按机器人半径做障碍膨胀
  -> 起点/终点吸附到最近自由栅格
  -> 8 邻域 A*
  -> 直线可通行路径简化
  -> /nav/path
```

## 4. 为什么第一阶段使用 2D/2.5D

当前小车底盘的运动自由度是 `x/y/yaw`，全局规划应优先解决平面可达性。参考 `demo/jie_3d_nav` 的 3D OctoMap A* 更适合完整三维空间搜索，但第一阶段直接上 3D 体素规划会带来三个问题：

- 地面小车最终仍要输出平面路径和速度，3D 搜索结果还需要投影、过滤和轨迹约束。
- 网页端第一目标是稳定显示地图、定位、目标点和全局路线。
- 动态障碍、近距离避障更适合放在局部 costmap 或点云安全层中处理。

因此第一阶段采用：

```text
2D 栅格全局规划 + 后续 3D 点云局部避障
```

后续可以把 `/luxi_localization/aligned_cloud_floor` 或 OctoMap 接入局部代价地图，用于动态障碍停车、绕障或局部重规划。

## 5. ROS2 接口

### 输入话题

```text
/map                    nav_msgs/msg/OccupancyGrid
/localization_2d         geometry_msgs/msg/PoseStamped
/Odometry                nav_msgs/msg/Odometry
/web/goal_pose           geometry_msgs/msg/PoseStamped
```

说明：

- `/localization_2d` 是正式定位输入，坐标系应与 `/map` 一致。
- `/Odometry` 只用于调试兜底，正式导航时建议关闭 `allow_odom_fallback`。
- `/web/goal_pose` 由网页点击目标点或 HTTP 接口发布。

### 输出话题

```text
/nav/path                nav_msgs/msg/Path
/nav/status              std_msgs/msg/String
/nav/cmd_vel             geometry_msgs/msg/Twist
```

`/nav/cmd_vel` 是默认 dry-run 输出，不接底盘。确认局部规划稳定后，可把 `cmd_vel_topic` 改成 `/cmd_vel`，或者让安全节点订阅 `/nav/cmd_vel` 后再转发到底盘。

`/nav/status` 是 JSON 字符串，包含：

```json
{
  "state": "path_ready",
  "detail": "规划成功 raw_cells=41 waypoints=3 start=(132,484) goal=(172,484)",
  "map_ready": true,
  "has_map_pose": false,
  "has_odom_pose": true,
  "has_goal": true,
  "has_path": true,
  "controller_enabled": true,
  "cmd_vel_topic": "/nav/cmd_vel",
  "path_topic": "/nav/path"
}
```

常见状态：

```text
waiting_map       等待地图
map_ready         地图已加载
waiting_pose      等待定位或里程计
waiting_goal      等待网页目标点
planning          正在规划
path_ready        规划成功
plan_failed       未找到路径
plan_timeout      规划超时
start_blocked     起点附近不可通行
goal_blocked      目标点附近不可通行
start_outside_map 起点不在地图范围内
goal_outside_map  目标点不在地图范围内
```

## 6. HTTP 接口

网页主要使用已有 `web_api_node`：

```text
GET  /api/map/status
GET  /api/map/snapshot
POST /api/goal_pose
```

目标点示例：

```bash
curl -X POST http://10.42.0.149:8082/api/goal_pose \
  -H 'Content-Type: application/json' \
  -d '{"x":2.053,"y":0.06,"yaw":0.0}'
```

路径验证：

```bash
curl http://10.42.0.149:8082/api/map/status
```

成功时应看到：

```json
{
  "has_nav_path": true,
  "nav_path_points": 3
}
```

## 7. 配置参数

配置文件：

```text
project/luxi-navigation/config/navigation.yaml
```

关键参数：

```yaml
map_topic: /map
goal_topic: /web/goal_pose
pose_topic: /localization_2d
odom_topic: /Odometry
path_topic: /nav/path
status_topic: /nav/status
frame_id: map

allow_odom_fallback: true
occupied_threshold: 65
unknown_is_occupied: true
robot_radius_m: 0.00
extra_inflation_m: 0.00
start_snap_max_m: 2.00
goal_snap_max_m: 1.00
max_iterations: 900000
max_plan_time_sec: 3.0
replan_on_goal: true
replan_on_pose_update: false
min_replan_period_sec: 0.5

controller_enabled: true
cmd_vel_topic: /nav/cmd_vel
controller_frequency: 10.0
max_vel_x: 0.22
min_vel_x: 0.0
max_vel_theta: 0.75
dwb_linear_samples: 6
dwb_angular_samples: 13
dwb_sim_time: 1.4
dwb_time_step: 0.1
dwb_path_distance_bias: 8.0
dwb_goal_distance_bias: 4.0
dwb_heading_bias: 1.5
dwb_velocity_bias: 0.4
dwb_stopped_trajectory_penalty: 2.0
dwb_collision_check_radius: 0.05
goal_tolerance_xy: 0.18
goal_tolerance_yaw: 0.35
lookahead_distance: 0.60
```

当前 `dwb_collision_check_radius` 为 dry-run 半径，用于在现有静态栅格噪声较多时验证局部轨迹采样链路。实车闭环前应结合机器人 footprint、局部点云 costmap 和安全停车逻辑重新调大。

参数建议：

- 第一阶段只做网页路线显示，默认关闭障碍膨胀，规划直接基于 `/map` 原始栅格。
- 正式接底盘闭环控制前，`robot_radius_m + extra_inflation_m` 应设置为大于真实车体外接半径和定位误差。
- `start_snap_max_m` 和 `goal_snap_max_m` 用于把轻微落在占用格上的起点/目标点吸附到附近可通行栅格。
- `unknown_is_occupied=true` 更安全，但地图未知区域较多时可能规划失败。
- 正式接底盘前建议设置 `allow_odom_fallback=false`，强制使用地图定位。
- `replan_on_pose_update=false` 可以避免每帧定位都触发 A*，第一阶段只在新目标点到来时规划。

## 8. 启动方式

完整启动：

```bash
cd /home/nvidia/project/luxi-atlas
source install/setup.bash
ros2 launch luxi_web_control web_navigation.launch.py
```

手机和设备在同一局域网时访问：

```text
http://10.42.0.149:8080
```

单独启动导航节点：

```bash
cd /home/nvidia/project/luxi-atlas
source install/setup.bash
ros2 launch luxi_navigation navigation.launch.py
```

## 9. 本地测试记录

构建命令：

```bash
cd /home/nvidia/project/luxi-atlas
colcon build --packages-select luxi_navigation luxi_web_control
```

启动后确认：

```bash
curl http://10.42.0.149:8082/api/map/status
```

当前测试结果：

```text
has_grid=true
has_static_cloud=true
has_odom_pose=true
has_nav_path=true
nav_path_points=3
nav_status.state=path_ready
```

`/nav/path` 当前测试路径点：

```text
(0.053, 0.060)
(1.653, -0.040)
(2.053, 0.060)
```

## 10. 当前限制

第一阶段不包含：

- 路径跟踪控制。
- `/cmd_vel` 输出。
- 动态障碍避障。
- 局部重规划。
- 路径速度曲线。
- 到点判定和任务状态机。

定位注意事项：

- 如果 `/localization_2d` 未初始化，当前会使用 `/Odometry` 兜底。
- `/Odometry` 的 `frame_id` 当前可能是 `camera_init`，这只适合调试。
- 正式导航必须保证 `/localization_2d`、`/map`、`/web/goal_pose` 在同一个地图坐标系下。

## 11. 后续阶段设计

第二阶段建议新增 `luxi_path_follower_node`：

```text
/nav/path
/localization_2d
  -> pure pursuit 或 Stanley 控制
  -> /nav/cmd_vel
```

再由控制仲裁节点决定：

```text
网页手动 /web/accepted_cmd_vel
导航自动 /nav/cmd_vel
急停 /web/estop
  -> 仲裁
  -> /cmd_vel
```

第三阶段建议新增局部安全层：

```text
/luxi_localization/aligned_cloud_floor
  -> 高度过滤
  -> 近场障碍检测
  -> 局部 costmap
  -> 停车/减速/局部绕障
```

第四阶段再考虑接入 3D OctoMap：

```text
PointCloud2 -> OctoMap/Voxel Map -> 2D 投影 costmap 或 3D A*
```

只有在存在坡道、多层结构、复杂高度约束时，才建议把 3D 体素规划作为主全局规划器。
