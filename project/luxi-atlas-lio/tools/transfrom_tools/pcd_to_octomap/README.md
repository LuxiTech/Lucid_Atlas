# PCD to OctoMap Converter

将 FAST-LIO2 保存的 `.pcd` 点云地图转换成 OctoMap `.bt` 或 `.ot`。

```text
PCD
  -> PointCloud
  -> Voxel Filtering
  -> OctoMap
  -> map.bt / map.ot
```

## 构建

```bash
cd /home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/tools/transfrom_tools/pcd_to_octomap
cmake -S . -B build
cmake --build build -j$(nproc)
```

## 转换最新 PCD

当前这张 `fast_lio_map_20260622_162740.pcd` 没有和世界 `z` 轴对齐，不要直接用 `--min-z/--max-z` 裁剪，否则会只保留一小块错误地图。OctoMap 建议先输出完整点云占据树：

```bash
cd /home/nvidia/project/luxi-atlas/project/luxi-atlas-lio

LATEST_PCD=$(ls -t PCD/*.pcd | head -1)
BASE=$(basename "$LATEST_PCD" .pcd)

LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libusb-1.0.so.0 \
  tools/transfrom_tools/pcd_to_octomap/build/pcd2octomap \
  --input "$LATEST_PCD" \
  --resolution 0.05 \
  --voxel-leaf-size 0.05 \
  --output "tools/transfrom_tools/pcd_to_octomap/output/${BASE}_full.bt"
```

输出：

```text
output/<map_name>_full.bt
output/<map_name>_full.bt.json
```

## 检查 OctoMap

```bash
tools/transfrom_tools/pcd_to_octomap/build/inspect_octomap \
  "tools/transfrom_tools/pcd_to_octomap/output/${BASE}_full.bt"
```

输出会包含分辨率、节点数、占据叶节点数和三维包围盒。

## 查看 OctoMap

先确认最新输出：

```bash
cd /home/nvidia/project/luxi-atlas/project/luxi-atlas-lio

LATEST_BT=$(ls -t tools/transfrom_tools/pcd_to_octomap/output/*.bt | head -1)
echo "$LATEST_BT"

tools/transfrom_tools/pcd_to_octomap/build/inspect_octomap "$LATEST_BT"
```

使用 OctoMap 自带查看器：

```bash
octovis "$LATEST_BT"
```

如果提示没有图形环境，先检查：

```bash
echo $DISPLAY
ls -la /tmp/.X11-unix
```

也可以在有桌面环境的电脑上打开同一个 `.bt` 文件。

在 ROS2/RViz 中查看：

```bash
sudo apt install -y ros-humble-octomap-server ros-humble-octomap-rviz-plugins

source /opt/ros/humble/setup.bash

ros2 run octomap_server octomap_server_node \
  --ros-args \
  -p frame_id:=map \
  -p octomap_path:="$LATEST_BT"
```

然后打开 RViz2，添加 `Octomap` 显示，话题选择 `/octomap_binary` 或 `/octomap_full`。

## 转完整 `.ot`

```bash
convert_octree \
  "tools/transfrom_tools/pcd_to_octomap/output/${BASE}_full.bt" \
  "tools/transfrom_tools/pcd_to_octomap/output/${BASE}_full.ot"
```

## 高度裁剪

只有当 PCD 已经确认与世界坐标对齐，即地面接近 `z=常数`，才使用：

```bash
LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libusb-1.0.so.0 \
  tools/transfrom_tools/pcd_to_octomap/build/pcd2octomap \
  --input "$LATEST_PCD" \
  --resolution 0.05 \
  --voxel-leaf-size 0.05 \
  --min-z -0.5 \
  --max-z 2.5 \
  --output "tools/transfrom_tools/pcd_to_octomap/output/${BASE}_crop.bt"
```

## 注意事项

- `.bt` 是二进制 occupied tree，文件小，适合常规查看和加载。
- `.ot` 是完整 tree，保留更多信息，文件通常更大。
- FAST-LIO 保存的 PCD 只有累计点云，没有逐帧传感器位姿；默认转换只把体素设为 occupied，没有可靠 free-space。
- 这张地图要生成 Nav2 的 PGM，请优先使用 `pcd_to_pgm` 的 `--floor-plane` 模式，而不是简单 `z` 轴裁剪。
