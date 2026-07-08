# RViz 定位显示例程

## 实现思路

定位节点负责发布两类信息：

```text
/luxi_localization/map_cloud_floor   投影到 2D 栅格平面的离线 PCD 地图
/luxi_localization/aligned_cloud_floor 投影到 2D 栅格平面的实时累积点云
/luxi_localization/debug_json  ICP 诊断信息
/localization_3d               当前 body 在 map 中的位姿
/localization_2d               当前 body 投影到 2D 栅格中的位姿
/map                           2D 导航栅格地图
TF: map -> camera_init         地图坐标系到 FAST-LIO 局部坐标系
```

RViz 只负责显示这些结果：

```text
Fixed Frame: map
PointCloud2: /luxi_localization/map_cloud_floor
PointCloud2: /luxi_localization/aligned_cloud_floor
Pose: /localization_3d
Map: /map
Pose: /localization_2d
TF: map -> camera_init -> body
```

定位节点默认需要先收到 `/initialpose`，然后才开始 ICP。这样可以避免在长直走廊中整图盲匹配到错误位置。

注意：RViz 的 `2D Pose Estimate` 现在按投影后的 2D 栅格坐标解释，也就是直接在 `/map` 和 `/luxi_localization/map_cloud_floor` 上点击。节点会根据 `floor_plane` 自动反投影回原始 3D PCD 地图，再补出高度和姿态。

初始化时还会拟合当前实时点云里的地面平面：

```text
/cloud_registered_body
-> 转到 camera_init/odom
-> 累积多帧点云
-> RANSAC 拟合当前扫描地面法向
-> 对齐到离线地图 floor_plane 法向
-> 结合 RViz 的 x/y/yaw 生成初始 map -> camera_init
```

这一步用于补偿雷达倾斜安装、本次 FAST-LIO 启动坐标系和离线地图坐标系之间的 roll/pitch 差异。

所以例程分成两个 launch：

```text
display_localization.launch.py   只启动定位节点和 RViz
bringup_localization.launch.py   启动 MID360、FAST-LIO、定位节点和 RViz
```

## 例程位置

```text
launch/bringup_localization.launch.py
launch/display_localization.launch.py
rviz/luxi_localization.rviz
```

`display_localization.launch.py` 做两件事：

```text
1. 启动 luxi_localization/localization_node
2. 启动 rviz2，并加载 rviz/luxi_localization.rviz
```

`bringup_localization.launch.py` 做四件事：

```text
1. 启动 livox_ros_driver2/msg_MID360_launch.py
2. 启动 fast_lio/mapping.launch.py
3. 启动 luxi_localization/localization_node
4. 启动 rviz2，并加载 rviz/luxi_localization.rviz
```

## 使用前提

先启动 MID360 和 FAST-LIO，并确认有数据：

```bash
ros2 topic info /Odometry
ros2 topic info /cloud_registered_body
ros2 run tf2_ros tf2_echo camera_init body
```

`/Odometry` 和 `/cloud_registered_body` 的 `Publisher count` 必须大于 0。

## 一键启动完整例程

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

## 只启动定位和 RViz

```bash
cd /home/nvidia/project/luxi-atlas/project/luxi-localization
source /opt/ros/humble/setup.bash
source /home/nvidia/project/luxi-atlas/device/mid360/install/setup.bash
source install/setup.bash

ros2 launch luxi_localization display_localization.launch.py \
  pcd_map:=/home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/PCD/fast_lio_map_20260622_162740.pcd \
  pgm_map:=/home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/tools/transfrom_tools/pcd_to_pgm/output/fast_lio_map_20260622_162740_nav_floor_plane_clean20/map.pgm
```

如果只想启动定位节点，不打开 RViz：

```bash
ros2 launch luxi_localization display_localization.launch.py \
  use_rviz:=false \
  pcd_map:=/home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/PCD/fast_lio_map_20260622_162740.pcd
```

## 查看输出

```bash
ros2 topic echo /luxi_localization/status --once
ros2 topic echo /localization_3d --once
ros2 topic echo /localization_2d --once
ros2 topic echo /localization_3d_confidence --once
ros2 topic echo /luxi_localization/debug_json --once
ros2 topic info /map
ros2 run tf2_ros tf2_echo map camera_init
```

## RViz 中应该看到

```text
蓝色投影点云地图: /luxi_localization/map_cloud_floor
灰白栅格地图: /map
黄色投影实时点云: /luxi_localization/aligned_cloud_floor
红色箭头: /localization_3d，默认隐藏
绿色箭头: /localization_2d
TF 坐标轴: map -> camera_init -> body
```

PCD 原始地图是 FAST-LIO 的倾斜三维坐标，PGM 栅格是按 `floor_plane` 投影后的地面坐标。RViz 默认显示 `*_floor` 话题，避免原始 PCD 和 2D 栅格叠加时上下分离。

投影 PCD 默认只显示 `0.15~1.60m` 的障碍高度切片，和生成当前 PGM 时使用的高度范围一致。

如果没有红色箭头，先看状态：

```bash
ros2 topic echo /luxi_localization/status --once
```

常见原因：

```text
waiting for odometry              FAST-LIO 没有发布 /Odometry
waiting for point cloud           FAST-LIO 没有发布 /cloud_registered_body
waiting for live floor plane      实时点云还没有拟合出稳定地面
waiting for initial pose          还没有在 RViz 中使用 2D Pose Estimate
ICP rejected                      初始位置偏差过大或点云匹配质量不足
```

在 RViz 顶部工具栏选择 `2D Pose Estimate`，在 2D 栅格或蓝色投影点云地图上给一个粗略初始位姿。初始位姿越接近真实位置，ICP 越不容易跳到相似走廊。

也可以用命令行给初始位姿。这里的 `x/y/yaw` 是当前 RViz 2D 栅格坐标：

```bash
ros2 run luxi_localization set_initial_pose 29.7 -9.0 0
```
