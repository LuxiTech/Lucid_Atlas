# OctoMap to OccupancyGrid Converter

将 OctoMap `.bt` / `.ot` 三维占据地图投影成 ROS1 `move_base` 和 ROS2 Nav2 常用的二维导航地图。

```text
OctoMap (.bt/.ot)
  -> z height filtering
  -> XY projection
  -> OccupancyGrid
  -> map.pgm + map.yaml
```

## 项目结构

```text
octomap_to_occupancygrid/
├── CMakeLists.txt
├── README.md
├── output/
└── src/
    └── octomap_to_occupancygrid.cpp
```

## 构建

```bash
cd /root/ws/catkin_ws/src/FAST_LIO_ROS2/tools/transfrom_tools/octomap_to_occupancygrid
cmake -S . -B build
cmake --build build -j
```

构建后生成：

```text
build/octomap2grid
```

## 快速转换

以前一步 `pcd_to_octomap` 生成的 OctoMap 为例：

```bash
./build/octomap2grid \
  --input /root/ws/catkin_ws/src/FAST_LIO_ROS2/tools/transfrom_tools/pcd_to_octomap/output/room_001.bt \
  --resolution 0.05 \
  --output /root/ws/catkin_ws/src/FAST_LIO_ROS2/tools/transfrom_tools/octomap_to_occupancygrid/output/room_001
```

生成：

```text
output/room_001/map.pgm
output/room_001/map.yaml
output/room_001/map.json
```

## 常用参数

```text
--output DIR
```

输出目录。工具会在该目录中生成 `map.pgm`、`map.yaml`、`map.json`。

```text
--resolution 0.05
```

输出二维 OccupancyGrid 分辨率。默认使用 OctoMap 分辨率。

```text
--occupied-threshold 0.65 --free-threshold 0.196
```

写入 `map.yaml` 的 ROS 地图阈值。

```text
--min-z -0.20 --max-z 1.20
```

只把指定高度范围内的 voxel 投影到 XY 平面。用于排除天花板、高处管线或地面以下噪声。

```text
--inflate-radius 0.20
```

离线膨胀 occupied cells。正式导航时仍建议在 Nav2 costmap 中配置 inflation layer。

```text
--unknown-as-free
```

将 unknown cells 写成 free。注意：如果 OctoMap 是由全局 PCD 直接转换而来，通常只有 occupied 信息，没有可靠 free 信息；启用此参数更方便做 2D 导航试验，但语义上不是严格的“已观测空闲区域”。

## 面向导航的示例

```bash
./build/octomap2grid \
  --input /root/ws/catkin_ws/src/FAST_LIO_ROS2/tools/transfrom_tools/pcd_to_octomap/output/room_001.bt \
  --resolution 0.05 \
  --min-z -0.20 \
  --max-z 1.20 \
  --inflate-radius 0.20 \
  --unknown-as-free \
  --output /root/ws/catkin_ws/src/FAST_LIO_ROS2/tools/transfrom_tools/octomap_to_occupancygrid/output/room_001_nav
```

## 查看二维地图

直接查看图像：

```bash
xdg-open /root/ws/catkin_ws/src/FAST_LIO_ROS2/tools/transfrom_tools/octomap_to_occupancygrid/output/room_001/map.pgm
```

无图形界面时查看元数据：

```bash
cat /root/ws/catkin_ws/src/FAST_LIO_ROS2/tools/transfrom_tools/octomap_to_occupancygrid/output/room_001/map.json
```

检查 PGM 头部：

```bash
head -n 4 /root/ws/catkin_ws/src/FAST_LIO_ROS2/tools/transfrom_tools/octomap_to_occupancygrid/output/room_001/map.pgm
```

## 在 RViz2 / Nav2 中查看

```bash
source /opt/ros/foxy/setup.bash

ros2 run nav2_map_server map_server \
  --ros-args \
  -p yaml_filename:=/root/ws/catkin_ws/src/FAST_LIO_ROS2/tools/transfrom_tools/octomap_to_occupancygrid/output/room_001_nav/map.yaml
```

另开终端激活生命周期节点：

```bash
source /opt/ros/foxy/setup.bash
ros2 run nav2_util lifecycle_bringup map_server
```

然后打开 RViz2，添加 `Map` 显示，话题选择 `/map`。

如果系统使用的是 ROS2 Humble / Jazzy，请将上面的 `/opt/ros/foxy/setup.bash` 改成实际版本路径。

## 注意事项

- occupied voxels 投影为黑色，free voxels 投影为白色，未观测区域投影为灰色。
- occupied 优先级高于 free，同一个二维 cell 同时被 free 和 occupied 投影覆盖时最终为 occupied。
- `.bt` / `.ot` 是三维占据地图；本工具输出的是二维投影地图，主要服务于 2D 路径规划、Nav2 静态地图加载和调试预览。
