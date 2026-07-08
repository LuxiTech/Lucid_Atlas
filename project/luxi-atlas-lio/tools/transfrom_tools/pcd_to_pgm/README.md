# PCD to PGM Converter

将 FAST-LIO2 保存的 `.pcd` 点云地图直接投影成 Nav2 可加载的二维栅格地图。

参考流程：

```text
PCD
  -> optional voxel downsample
  -> z height filtering
  -> optional radius outlier removal
  -> XY projection
  -> map.pgm + map.yaml
```

这个工具参考 `LihanChen2004/pcd2pgm` 的处理思路：读取 PCD、按高度过滤点云、可选半径离群点滤波，再生成用于 Navigation 的 PGM 栅格地图。

## 构建

```bash
cd /home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/tools/transfrom_tools/pcd_to_pgm
cmake -S . -B build
cmake --build build -j$(nproc)
```

## 转换最新 PCD

```bash
cd /home/nvidia/project/luxi-atlas/project/luxi-atlas-lio

LATEST_PCD=$(ls -t PCD/*.pcd | head -1)
BASE=$(basename "$LATEST_PCD" .pcd)

LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libusb-1.0.so.0 \
  tools/transfrom_tools/pcd_to_pgm/build/pcd2pgm \
  --input "$LATEST_PCD" \
  --floor-plane -0.43606115 0.20801629 0.87554548 1.24319933 \
  --resolution 0.05 \
  --voxel-leaf-size 0.05 \
  --min-z 0.05 \
  --max-z 0.50 \
  --min-component-cells 80 \
  --inflate-radius 0.05 \
  --output "tools/transfrom_tools/pcd_to_pgm/output/${BASE}_nav_floor_plane_h005_050_clean80"
```

输出：

```text
output/<map_name>_nav_floor_plane_h005_050_clean80/map.pgm
output/<map_name>_nav_floor_plane_h005_050_clean80/map.yaml
output/<map_name>_nav_floor_plane_h005_050_clean80/map.json
```

## 常用参数

```text
--resolution 0.05
```

输出二维地图分辨率。

```text
--min-z 0.05 --max-z 0.50
```

只保留指定离地高度范围内的点。使用 `--floor-plane` 时，这里的高度是点到地面平面的距离，不是原始世界坐标 z。当前导航测试按底盘可碰撞高度处理：距离地面 `0.05~0.50m` 的点标记为障碍，地面本身和更高的天花板/上层结构不写入 2D 占据栅格。

```text
--floor-plane -0.43606115 0.20801629 0.87554548 1.24319933
```

使用拟合出的地面平面作为导航投影平面，`--min-z` / `--max-z` 表示离地高度。当前 `fast_lio_map_20260622_162740.pcd` 的地面并不平行于世界 `z=0`，必须使用该参数，否则会切到天花板或只剩一小块地图。

```text
--height-axis x|y|z
```

简单指定哪个轴作为高度轴。只适合地图已经和世界坐标对齐的 PCD；当前这张地图不建议用它作为最终导航图。

```text
--voxel-leaf-size 0.05
```

点云下采样尺寸。设为 `0` 可关闭。

```text
--radius-outlier --radius-search 0.10 --min-neighbors 10
```

启用半径离群点滤波，去除孤立点。

```text
--inflate-radius 0.20
```

离线膨胀障碍物。正式导航时仍建议在 Nav2 costmap 中配置 inflation layer。

```text
--min-component-cells 80
```

删除小于指定栅格数量的 2D 障碍物连通域，用于去掉人腿、反光噪声、累计点云中的临时小障碍。值越大，去噪越强，但也可能误删细小真实结构。当前地图推荐先用 `80`。

```text
--keep-unknown
```

默认会把非障碍区域写成 free，方便直接做 2D 导航试验。加上该参数后，非障碍区域保持 unknown。

## Nav2 加载

```bash
ros2 run nav2_map_server map_server \
  --ros-args \
  -p yaml_filename:=/home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/tools/transfrom_tools/pcd_to_pgm/output/<map_name>_nav/map.yaml
```

另开终端激活：

```bash
ros2 run nav2_util lifecycle_bringup map_server
```

## 注意事项

- PCD 只有 occupied 点，没有 ray casting 得到的 free 信息；默认 `unknown_as_free` 是为了方便导航试验，不代表严格观测空闲区。
- 直接 PCD 投影比 `PCD -> OctoMap -> OccupancyGrid` 更快，但三维占据信息更少。
- 如果地图障碍太厚或太碎，优先调 `--min-z`、`--max-z`、`--voxel-leaf-size` 和 `--inflate-radius`。

## map5 栅格图黑点说明

`/home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/maps/test_map5.pcd` 转出的旧图：

```text
tools/transfrom_tools/pcd_to_pgm/output/test_map5_nav_floor_plane_autofit_clean20/map.pgm
```

有明显的大块黑点。黑色在 Nav2 的 PGM 中表示 occupied，也就是障碍物；它不是显示错误，也不是 unknown 区域。

旧图的 `map.json` 关键数据：

```text
floor_plane: [-0.357093, 0.006594, 0.934045, 1.001687]
min_z_filter: 0.15
max_z_filter: 1.60
height_filtered_points: 86308
occupied_cells_before_inflation: 23578
occupied_cells_after_component_filter: 19731
occupied_cells_after_inflation: 33991
removed_components: 1673
removed_component_cells: 3847
```

结论：

- 这次确实使用了自动地面拟合，`projection_plane` 是 `floor_plane`，不是简单按世界 z 投影。
- 大黑点的直接原因是有大量点落在 `0.15~1.60m` 的障碍高度带内，转换器会把这些点全部投影成 occupied。
- 这不一定代表“地面完全没拟合成功”。更准确地说，`0.15m` 下限太低，地面起伏、地面反射、低矮杂物、机器人或人经过留下的动态点云残留，都可能进入障碍高度带。
- 自动拟合平面也可能有局部误差。只要某一片地面点相对拟合平面偏高超过 `0.15m`，就会被当成障碍写成黑点。

后续基于 `map5` 做导航，按底盘可碰撞高度重新生成：

```bash
cd /home/nvidia/project/luxi-atlas/project/luxi-atlas-lio

LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libusb-1.0.so.0 \
  tools/transfrom_tools/pcd_to_pgm/build/pcd2pgm \
  --input maps/test_map5.pcd \
  --floor-plane -0.357093 0.006594 0.934045 1.001687 \
  --resolution 0.05 \
  --voxel-leaf-size 0.05 \
  --min-z 0.05 \
  --max-z 0.50 \
  --min-component-cells 80 \
  --inflate-radius 0.05 \
  --output tools/transfrom_tools/pcd_to_pgm/output/test_map5_nav_floor_plane_autofit_h005_050_clean80
```

这组参数只把离地 `0.05~0.50m` 的点写成 occupied。正式导航前必须在 RViz 或网页中确认机器人路径没有被黑色 occupied 区域切断；如果通道仍被黑点堵住，需要重新采图、调整高度带，或对 PGM 做人工清图后再交给 Nav2。
