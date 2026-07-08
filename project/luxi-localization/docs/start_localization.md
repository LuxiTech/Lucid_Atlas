# Luxi Localization 启动与查看

目标链路：

```text
MID360 -> FAST-LIO -> luxi_localization -> RViz
```

定位使用的地图：

```bash
/home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/PCD/fast_lio_map_20260622_162740.pcd
```

2D 栅格地图：

```bash
/home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/tools/transfrom_tools/pcd_to_pgm/output/fast_lio_map_20260622_162740_nav_floor_plane_clean20/map.pgm
```

## 1. 构建定位包

```bash
cd /home/nvidia/project/luxi-atlas/project/luxi-localization
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 2. 启动 MID360

终端 1：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/project/luxi-atlas/device/mid360/install/setup.bash
export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libusb-1.0.so.0

ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

## 3. 启动 FAST-LIO

终端 2：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/project/luxi-atlas/device/mid360/install/setup.bash
export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libusb-1.0.so.0

ros2 launch fast_lio mapping.launch.py \
  config_path:=/home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/config \
  config_file:=mid360.yaml \
  rviz:=false
```

确认 FAST-LIO 有输出：

```bash
ros2 topic info /Odometry
ros2 topic info /cloud_registered_body
ros2 run tf2_ros tf2_echo camera_init body
```

`/Odometry` 和 `/cloud_registered_body` 的 `Publisher count` 必须大于 0。

## 4. 启动定位节点

终端 3：

```bash
cd /home/nvidia/project/luxi-atlas/project/luxi-localization
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch luxi_localization localization.launch.py \
  pcd_map:=/home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/PCD/fast_lio_map_20260622_162740.pcd
```

查看定位状态：

```bash
ros2 topic echo /luxi_localization/status --once
```

常见状态：

```text
waiting for odometry: /Odometry
waiting for point cloud: /cloud_registered_body
waiting for live floor plane
waiting for initial pose: /initialpose
localized: confidence=...
ICP rejected: fitness=... rmse=...
```

现在默认 `require_initial_pose=true`，没有初始位姿时不会做 ICP，避免在长走廊或重复结构中盲匹配到错误位置。

初始位姿给法：

```text
RViz -> 2D Pose Estimate
```

在 2D 栅格地图或蓝色投影点云地图上点出机器人当前的大概位置和朝向。该位姿默认是投影后的 floor 坐标，节点会自动反投影回原始 3D PCD 地图，然后计算 `map -> camera_init`。

注意：RViz 只给 `x/y/yaw`，节点会根据 `config/localization.yaml` 中的 `floor_plane` 自动恢复 3D PCD 地面的高度和姿态。当前默认地面平面来自这张地图的拟合结果。

同时节点会在初始化时拟合当前实时扫描的地面平面，并把当前扫描地面法向对齐到离线地图地面法向。这样可以补偿雷达倾斜安装导致的本次 `camera_init` roll/pitch 和地图坐标系不一致的问题。

## 5. 查看定位输出

终端 4：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/project/luxi-atlas/project/luxi-localization/install/setup.bash

ros2 topic echo /localization_3d --once
ros2 topic echo /localization_2d --once
ros2 topic echo /localization_3d_confidence --once
ros2 topic echo /luxi_localization/debug_json --once
ros2 topic info /map
ros2 run tf2_ros tf2_echo map camera_init
```

输出含义：

```text
/localization_3d              当前 body 在 map 中的位姿
/localization_3d_confidence   ICP 匹配置信度
/luxi_localization/debug_json ICP 诊断信息
map -> camera_init            全局地图到 FAST-LIO 局部坐标系的 TF
```

## 6. RViz 显示

推荐直接使用完整例程，同时启动 MID360、FAST-LIO、定位节点和 RViz：

```bash
cd /home/nvidia/project/luxi-atlas/project/luxi-localization
source /opt/ros/humble/setup.bash
source /home/nvidia/project/luxi-atlas/device/mid360/install/setup.bash
source install/setup.bash
export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libusb-1.0.so.0

ros2 launch luxi_localization bringup_localization.launch.py \
  pcd_map:=/home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/PCD/fast_lio_map_20260622_162740.pcd \
  pgm_map:=/home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/tools/transfrom_tools/pcd_to_pgm/output/fast_lio_map_20260622_162740_nav_floor_plane_clean20/map.pgm
```

`pgm_map` 可以传 `map.pgm`，也可以传对应的 `map.yaml`。传 `map.pgm` 时会自动读取同目录的 `map.yaml` 获取分辨率和原点。

如果已经手动启动了 MID360 和 FAST-LIO，只启动定位和 RViz：

```bash
ros2 launch luxi_localization display_localization.launch.py \
  pcd_map:=/home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/PCD/fast_lio_map_20260622_162740.pcd \
  pgm_map:=/home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/tools/transfrom_tools/pcd_to_pgm/output/fast_lio_map_20260622_162740_nav_floor_plane_clean20/map.pgm
```

详细说明见：

```text
docs/rviz_localization_example.md
```

也可以手动打开 RViz：

```bash
rviz2
```

RViz 设置：

```text
Fixed Frame: map

Add -> PointCloud2
Topic: /luxi_localization/map_cloud_floor

Add -> TF

Add -> Pose
Topic: /localization_2d

Add -> PointCloud2
Topic: /luxi_localization/aligned_cloud_floor
```

显示效果：

```text
/luxi_localization/map_cloud_floor 显示投影到 2D 栅格平面的离线 PCD 地图
/map                          显示 2D 栅格地图
/luxi_localization/aligned_cloud_floor 显示投影到 2D 栅格平面的实时累积点云
TF                            显示 map -> camera_init -> body
/localization_3d              显示原始 3D 当前位置，默认不作为 RViz 主显示
/localization_2d              显示投影到栅格地图的当前位置箭头
```

默认 RViz 使用 `floor_projected_cloud_min_height: 0.15` 和 `floor_projected_cloud_max_height: 1.60`，只显示和 PGM 生成一致的障碍高度切片。

如果没有当前位置，先看：

```bash
ros2 topic echo /luxi_localization/status --once
ros2 topic info /Odometry
ros2 topic info /cloud_registered_body
```

## 7. 可选：给初始位姿

必须先在 RViz 使用 `2D Pose Estimate` 给 `/initialpose` 一个粗略初始位姿。

本节点会根据 `/initialpose` 修正初始的 `map -> camera_init`。默认配置 `initial_pose_coordinate_frame: floor`，表示 RViz 发来的初始位姿在 2D 栅格/投影 PCD 坐标中。

也可以用命令行发布初始位姿。这里的 `x/y/yaw` 是当前 RViz 2D 栅格坐标：

```bash
ros2 run luxi_localization set_initial_pose 29.7 -9.0 0
```

如果要把 `/initialpose` 当成原始 3D PCD 的 map 坐标，可以在配置中改：

```yaml
initial_pose_coordinate_frame: map
```

如果你要复现参考程序的语义，即把 `/initialpose` 当成 `map -> camera_init` 而不是 `map -> body`，可以启动时设置：

```bash
ros2 launch luxi_localization bringup_localization.launch.py \
  pcd_map:=/home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/PCD/fast_lio_map_20260622_162740.pcd \
  config_file:=/home/nvidia/project/luxi-atlas/project/luxi-localization/config/localization.yaml
```

并在配置中改：

```yaml
initial_pose_mode: odom
```
