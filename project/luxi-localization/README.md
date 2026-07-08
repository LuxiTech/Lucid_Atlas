# luxi-localization

基于 Open3D 的 FAST-LIO 全局定位节点。

当前定位链路：

```text
MID360 -> FAST-LIO -> /Odometry + /cloud_registered_body
                         |
                         v
                  luxi_localization
                         |
                         v
                 TF: map -> camera_init
```

## 目录

```text
luxi-localization/
├── config/localization.yaml
├── launch/localization.launch.py
├── luxi_localization/
│   ├── localization_node.py
│   ├── pointcloud.py
│   └── transforms.py
├── tools/offline_icp_check.py
├── tests/test_transforms.py
├── package.xml
└── setup.py
```

Open3D 源码固定在：

```bash
/home/nvidia/project/luxi-atlas/third_party/open3d
```

运行时使用系统 Python 环境里的 `open3d==0.18.0`。

## 构建

```bash
cd /home/nvidia/project/luxi-atlas/project/luxi-localization
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 离线测试

```bash
cd /home/nvidia/project/luxi-atlas/project/luxi-localization
python3 tools/offline_icp_check.py \
  /home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/PCD/fast_lio_map_20260622_162740.pcd
```

## 启动定位

先启动 MID360 和 FAST-LIO，再运行：

```bash
cd /home/nvidia/project/luxi-atlas/project/luxi-localization
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch luxi_localization localization.launch.py \
  map_path:=/home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/PCD/fast_lio_map_20260622_162740.pcd
```

## 检查输出

```bash
ros2 topic echo /localization_3d --once
ros2 topic echo /localization_3d_confidence --once
ros2 run tf2_ros tf2_echo map camera_init
```

## 输入输出

默认输入：

```text
/Odometry
/cloud_registered_body
/initialpose
```

默认输出：

```text
/luxi_localization/map_cloud
/luxi_localization/aligned_cloud
/luxi_localization/status
/luxi_localization/debug_json
/localization_3d
/localization_3d_confidence
TF: map -> camera_init
```

默认 `require_initial_pose=true`，需要先在 RViz 使用 `2D Pose Estimate` 给 `/initialpose` 一个粗略初始位姿。由于 FAST-LIO 保存的 PCD 地图地面不是 `z=0` 水平面，节点会把 2D 初始位姿自动投影到配置的 `floor_plane` 上。

初始化时还会对当前实时点云做 RANSAC 地面平面拟合，将当前扫描地面法向对齐到离线地图地面法向，再生成初始 `map -> camera_init`。这一步用于补偿雷达倾斜安装或本次 FAST-LIO 启动坐标系 roll/pitch 与旧地图不一致的问题。随后节点在该初始位姿附近裁剪局部 PCD 地图，并用累积实时点云做 ICP，避免在长走廊中整图盲匹配到错误位置。

## RViz 显示

先启动定位节点，然后打开 RViz：

```bash
rviz2
```

RViz 设置：

```text
Fixed Frame: map

Add -> PointCloud2
Topic: /luxi_localization/map_cloud

Add -> TF

Add -> Pose
Topic: /localization_3d

Add -> PointCloud2
Topic: /luxi_localization/aligned_cloud
```

如果 `/localization_3d` 没有输出，先检查状态：

```bash
ros2 topic echo /luxi_localization/status --once
ros2 topic info /Odometry
ros2 topic info /cloud_registered_body
```

`/Odometry` 和 `/cloud_registered_body` 必须有 Publisher，定位才会输出当前位置。

## 坐标约定

FAST-LIO 发布：

```text
camera_init -> body
```

本节点发布：

```text
map -> camera_init
```

组合后得到：

```text
map -> camera_init -> body
```
