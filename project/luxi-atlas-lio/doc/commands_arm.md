# FAST-LIO2 MID360 ARM Commands

设备环境：

- Ubuntu 22.04 / ROS2 Humble / aarch64 Jetson
- 主机网卡：`enP8p1s0`
- 主机 IP：`192.168.1.50`
- MID360 IP：`192.168.1.147`
- ROS2 工作区：`/home/nvidia/project/luxi-atlas/device/mid360`
- FAST-LIO2 项目：`/home/nvidia/project/luxi-atlas/project/luxi-atlas-lio`

## 1. 环境

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/project/luxi-atlas/device/mid360/install/setup.bash
export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libusb-1.0.so.0
```

检查：

```bash
echo $ROS_DISTRO
ros2 pkg prefix livox_ros_driver2
ros2 pkg prefix fast_lio
```

## 2. 查看原始 MID360

只看原始点云时使用：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/project/luxi-atlas/device/mid360/install/setup.bash
export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libusb-1.0.so.0

ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```

另开终端检查：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/project/luxi-atlas/device/mid360/install/setup.bash

ros2 topic list | grep livox
ros2 topic info /livox/lidar -v
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
```

## 3. 启动建图

FAST-LIO 必须使用 `msg_MID360_launch.py`，因为它发布带单点时间戳的 `livox_ros_driver2/CustomMsg`。

终端 1，启动驱动：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/project/luxi-atlas/device/mid360/install/setup.bash
export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libusb-1.0.so.0

ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

终端 2，启动 FAST-LIO。默认先不打开 RViz，稳定后再单独开 RViz。

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/project/luxi-atlas/device/mid360/install/setup.bash
export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libusb-1.0.so.0

PCD_DIR=/home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/PCD
CONFIG_DIR=/home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/runtime_config
MAP_NAME=fast_lio_map_$(date +%Y%m%d_%H%M%S).pcd

mkdir -p "$PCD_DIR" "$CONFIG_DIR"

sed \
  "s#map_file_path: .*#map_file_path: \"$PCD_DIR/$MAP_NAME\"#" \
  /home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/config/mid360.yaml \
  > "$CONFIG_DIR/mid360.yaml"

echo "PCD will be saved to: $PCD_DIR/$MAP_NAME"

ros2 launch fast_lio mapping.launch.py \
  config_path:="$CONFIG_DIR" \
  config_file:=mid360.yaml \
  rviz:=false
```

需要 RViz 时：

```bash
rviz2 -d /home/nvidia/project/luxi-atlas/device/mid360/install/fast_lio/share/fast_lio/rviz/fastlio.rviz
```

## 4. 检查建图输出

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/project/luxi-atlas/device/mid360/install/setup.bash

ros2 node list
ros2 topic list | grep -E 'Odometry|path|cloud_registered|Laser_map|livox'
ros2 param get /laser_mapping map_file_path
```

常用输出：

```text
/Odometry
/path
/cloud_registered
/cloud_registered_body
/Laser_map
```

查看位姿和 TF：

```bash
ros2 topic hz /Odometry
ros2 run tf2_ros tf2_echo camera_init body
```

坐标关系：

```text
camera_init -> body
```

`camera_init` 是本次启动的局部世界原点，`body` 是 LiDAR-IMU 机体系。

## 5. 保存 PCD 地图

建图过程中或结束前调用：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/project/luxi-atlas/device/mid360/install/setup.bash

ros2 service list | grep map_save
ros2 service call /map_save std_srvs/srv/Trigger {}
ls -lh /home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/PCD
```

`/map_save` 保存到启动时写入的 `map_file_path`。同一个 FAST-LIO 进程内重复保存会覆盖同一个文件。

## 6. 录制与回放

录制建图输入：

```bash
mkdir -p /home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/bags

ros2 bag record \
  /livox/lidar \
  /livox/imu \
  -o /home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/bags/mid360_$(date +%Y%m%d_%H%M%S)
```

查看和回放：

```bash
ros2 bag info /home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/bags/<bag_dir>
ros2 bag play /home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/bags/<bag_dir>
```

## 7. 查看 PCD

```bash
sudo apt update
sudo apt install -y pcl-tools

LATEST_PCD=$(ls -t /home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/PCD/*.pcd | head -1)
echo "$LATEST_PCD"

LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libusb-1.0.so.0 \
  pcl_viewer "$LATEST_PCD"
```

## 8. PCD 转 GLB

```bash
cd /home/nvidia/project/luxi-atlas/project/luxi-atlas-lio
python3 -m pip install -r tools/transfrom_tools/pcd_to_glb/requirements.txt

LATEST_PCD=$(ls -t PCD/*.pcd | head -1)
BASE=$(basename "$LATEST_PCD" .pcd)

python3 tools/transfrom_tools/pcd_to_glb/pcd_to_glb.py "$LATEST_PCD" \
  -o "tools/transfrom_tools/pcd_to_glb/output/${BASE}.glb" \
  --method voxel \
  --voxel-size 0.15 \
  --target-triangles 250000

python3 tools/transfrom_tools/pcd_to_glb/validate_glb.py \
  "tools/transfrom_tools/pcd_to_glb/output/${BASE}.glb"
```

## 9. PCD 直接转 Nav2 地图

```bash
cd /home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/tools/transfrom_tools/pcd_to_pgm
cmake -S . -B build
cmake --build build -j$(nproc)

cd /home/nvidia/project/luxi-atlas/project/luxi-atlas-lio

LATEST_PCD=$(ls -t PCD/*.pcd | head -1)
BASE=$(basename "$LATEST_PCD" .pcd)

LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libusb-1.0.so.0 \
  tools/transfrom_tools/pcd_to_pgm/build/pcd2pgm \
  --input "$LATEST_PCD" \
  --floor-plane -0.43606115 0.20801629 0.87554548 1.24319933 \
  --resolution 0.05 \
  --voxel-leaf-size 0.05 \
  --min-z 0.15 \
  --max-z 1.60 \
  --min-component-cells 80 \
  --inflate-radius 0.05 \
  --output "tools/transfrom_tools/pcd_to_pgm/output/${BASE}_nav_floor_plane_clean80"
```

输出：

```text
map.pgm
map.yaml
map.json
```

## 10. PCD 转 OctoMap

```bash
sudo apt update
sudo apt install -y liboctomap-dev octomap-tools octovis

cd /home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/tools/transfrom_tools/pcd_to_octomap
cmake -S . -B build
cmake --build build -j$(nproc)

LATEST_PCD=$(ls -t /home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/PCD/*.pcd | head -1)
echo "$LATEST_PCD"

LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libusb-1.0.so.0 \
  ./build/pcd2octomap \
  --input "$LATEST_PCD" \
  --resolution 0.05 \
  --voxel-leaf-size 0.05 \
  --output /home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/tools/transfrom_tools/pcd_to_octomap/output/fast_lio_map.bt
```

查看：

```bash
./build/inspect_octomap \
  /home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/tools/transfrom_tools/pcd_to_octomap/output/fast_lio_map.bt

octovis /home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/tools/transfrom_tools/pcd_to_octomap/output/fast_lio_map.bt
```

如需完整 `.ot`：

```bash
convert_octree \
  /home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/tools/transfrom_tools/pcd_to_octomap/output/fast_lio_map.bt \
  /home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/tools/transfrom_tools/pcd_to_octomap/output/fast_lio_map.ot
```

## 11. OctoMap 转 Nav2 地图

```bash
cd /home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/tools/transfrom_tools/octomap_to_occupancygrid
cmake -S . -B build
cmake --build build -j$(nproc)

./build/octomap2grid \
  --input /home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/tools/transfrom_tools/pcd_to_octomap/output/fast_lio_map.bt \
  --resolution 0.05 \
  --min-z -0.20 \
  --max-z 1.20 \
  --inflate-radius 0.20 \
  --unknown-as-free \
  --output /home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/tools/transfrom_tools/octomap_to_occupancygrid/output/fast_lio_map_nav
```

输出：

```text
map.pgm
map.yaml
map.json
```

加载到 Nav2 map server：

```bash
sudo apt install -y ros-humble-nav2-map-server ros-humble-nav2-lifecycle-manager ros-humble-nav2-util

source /opt/ros/humble/setup.bash

ros2 run nav2_map_server map_server \
  --ros-args \
  -p yaml_filename:=/home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/tools/transfrom_tools/octomap_to_occupancygrid/output/fast_lio_map_nav/map.yaml
```

另开终端激活：

```bash
source /opt/ros/humble/setup.bash
ros2 run nav2_util lifecycle_bringup map_server
```

## 12. 常见问题

### `fastlio_mapping` 报 `libusb_set_option`

说明加载到了相机 SDK 或其他旧版 `libusb`。启动前执行：

```bash
export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libusb-1.0.so.0
```

不要使用 x86_64 路径。

### 没有 `/Odometry`

先确认驱动是 `msg_MID360_launch.py`：

```bash
ros2 topic info /livox/lidar -v
ros2 topic hz /livox/imu
```

如果 `/livox/lidar` 不是 `livox_ros_driver2/msg/CustomMsg`，FAST-LIO 建图输入不对。

### 保存地图失败

检查当前 FAST-LIO 实际保存路径：

```bash
ros2 param get /laser_mapping map_file_path
ls -ld /home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/PCD
```

如果仍是 `/root/ws/...`，停止 FAST-LIO，按第 3 节重新生成 `runtime_config/mid360.yaml` 后再启动。

### `sequence size exceeds remaining buffer`

常见于 RViz 或 DDS 解析不匹配消息。先关 RViz，用第 3 节的 `rviz:=false` 跑建图；只要 `fastlio_mapping` 没退出，优先检查 `/Odometry` 和 `/Laser_map`。
