# PCD to GLB Converter

将 FAST-LIO2 保存的 `.pcd` 点云地图转换成可导入 Habitat-Sim 或三维查看器的 `.glb` 网格。

## 依赖

```bash
cd /home/nvidia/project/luxi-atlas/project/luxi-atlas-lio
python3 -m pip install -r tools/transfrom_tools/pcd_to_glb/requirements.txt
```

已验证环境：

```text
Python 3.10
open3d 0.18.0
trimesh 4.12.2
```

## 转换最新 PCD

```bash
cd /home/nvidia/project/luxi-atlas/project/luxi-atlas-lio

LATEST_PCD=$(ls -t PCD/*.pcd | head -1)
BASE=$(basename "$LATEST_PCD" .pcd)

python3 tools/transfrom_tools/pcd_to_glb/pcd_to_glb.py "$LATEST_PCD" \
  -o "tools/transfrom_tools/pcd_to_glb/output/${BASE}.glb" \
  --method voxel \
  --voxel-size 0.15 \
  --target-triangles 250000
```

输出：

```text
tools/transfrom_tools/pcd_to_glb/output/<map_name>.glb
tools/transfrom_tools/pcd_to_glb/output/<map_name>.glb.json
```

## 验证

```bash
python3 tools/transfrom_tools/pcd_to_glb/validate_glb.py \
  "tools/transfrom_tools/pcd_to_glb/output/${BASE}.glb"
```

验证通过时会输出文件大小、顶点数、面数、包围盒，并且 `is_empty` 为 `false`。

## 查看 GLB

先确认最新输出：

```bash
cd /home/nvidia/project/luxi-atlas/project/luxi-atlas-lio

LATEST_GLB=$(ls -t tools/transfrom_tools/pcd_to_glb/output/*.glb | head -1)
echo "$LATEST_GLB"

python3 tools/transfrom_tools/pcd_to_glb/validate_glb.py "$LATEST_GLB"
```

如果本机安装了 Blender，不要直接运行 `blender map.glb`，Blender 会把参数当成 `.blend` 工程文件。用 glTF importer 导入：

```bash
blender --python-expr "import bpy; bpy.ops.object.delete(); bpy.ops.import_scene.gltf(filepath='$LATEST_GLB')"
```

也可以先导入并保存成 `.blend`：

```bash
blender --background --python-expr "import bpy; bpy.ops.object.delete(); bpy.ops.import_scene.gltf(filepath='$LATEST_GLB'); bpy.ops.wm.save_as_mainfile(filepath='/tmp/fast_lio_map.blend')"

blender /tmp/fast_lio_map.blend
```

当前机器如果没有 `blender` 命令，可以安装：

```bash
sudo apt update
sudo apt install -y blender
```

也可以把 `.glb` 文件复制到桌面电脑，用 Blender、Windows 3D Viewer、macOS 预览、VS Code glTF/GLB 插件，或网页 glTF Viewer 打开。

注意：默认转换会把 ROS `z-up` 坐标转成 glTF/Habitat 常用的 `y-up`。如果希望查看时保留 ROS 坐标，转换时加：

```bash
python3 tools/transfrom_tools/pcd_to_glb/pcd_to_glb.py "$LATEST_PCD" \
  -o "tools/transfrom_tools/pcd_to_glb/output/${BASE}_ros.glb" \
  --method voxel \
  --voxel-size 0.15 \
  --target-triangles 250000 \
  --no-ros-to-habitat
```

## 常用参数

```bash
python3 tools/transfrom_tools/pcd_to_glb/pcd_to_glb.py "$LATEST_PCD" \
  -o "tools/transfrom_tools/pcd_to_glb/output/${BASE}.glb" \
  --method voxel \
  --voxel-size 0.15 \
  --target-triangles 250000
```

- `--method voxel`：最稳，适合稀疏 LiDAR 点云。
- `--voxel-size 0.15`：体素尺寸，数值越小模型越细、文件越大。
- `--target-triangles 250000`：面数上限，用于控制 GLB 大小。
- `--no-ros-to-habitat`：保留 ROS 坐标；默认会转换为 glTF/Habitat 的 y-up 坐标。

表面重建可选：

```bash
python3 tools/transfrom_tools/pcd_to_glb/pcd_to_glb.py "$LATEST_PCD" \
  -o "tools/transfrom_tools/pcd_to_glb/output/${BASE}_bpa.glb" \
  --method bpa \
  --voxel-size 0.08
```

`bpa` / `poisson` 更像连续表面，但对点云密度、噪声和法线更敏感。导航验证建议先用 `voxel`。
