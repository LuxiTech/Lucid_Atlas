# Luxi Device Launch

这里存放 Luxi 设备侧预览 launch 文件，用于在启动 FAST-LIVO2 算法前确认相机和雷达话题是否正常。

## 文件说明

- `cu013_camera_preview.launch.py`：只启动海康 `MV-CU013-A0UC` 相机节点和 RViz 预览。
- `device_cu013_mid360_preview.launch.py`：启动 `MV-CU013-A0UC` 相机、Livox `MID360` 雷达和 RViz 预览。
- `../luxi_device_cu013_mid360_preview.launch.py`：顶层入口文件，推荐用 `ros2 launch` 启动这个文件。

## 编译

不要在 `/home/nvidia/project/luxi-atlas` 顶层直接执行 `colcon build --packages-select fast_livo`，因为当前目录下同时存在两个名为 `fast_livo` 的包：

- `demo/lux-atlas`
- `project/luxi-atlas-livo`

请使用 `--base-paths` 只扫描需要编译的源码路径：

```bash
cd /home/nvidia/project/luxi-atlas
source /opt/ros/humble/setup.bash
source /home/nvidia/project/luxi-atlas/device/mid360/install/setup.bash
source /home/nvidia/project/luxi-atlas/device/luxi-hikrobot/ROS2/MV-CU013-A0UC/ws/install/setup.bash

colcon build --symlink-install --base-paths third_party/rpg_vikit project/luxi-atlas-livo
```

## 运行相机和 MID360 预览

```bash
cd /home/nvidia/project/luxi-atlas
source /opt/ros/humble/setup.bash
source /home/nvidia/project/luxi-atlas/device/mid360/install/setup.bash
source /home/nvidia/project/luxi-atlas/device/luxi-hikrobot/ROS2/MV-CU013-A0UC/ws/install/setup.bash
source /home/nvidia/project/luxi-atlas/install/setup.bash

ros2 launch fast_livo luxi_device_cu013_mid360_preview.launch.py
```

预期话题：

```text
/hikrobot/cu013/rgb_img  sensor_msgs/msg/Image
/livox/lidar             sensor_msgs/msg/PointCloud2
/livox/imu               sensor_msgs/msg/Imu
```

## FAST-LIVO2 输入模式

RViz 点云预览推荐使用默认值 `mid360_xfer_format:=0`，此时 `/livox/lidar` 是 `sensor_msgs/msg/PointCloud2`。

如果后续要给 FAST-LIVO2 算法层使用 Livox 自定义消息，启动时改为：

```bash
ros2 launch fast_livo luxi_device_cu013_mid360_preview.launch.py mid360_xfer_format:=1
```

## 外部触发测试

当前 Luxi 启动文件默认开启相机外部触发，使用 `Line0` 上升沿。相机 `Line0` 接 STM32 的 10Hz PWM 时，直接启动即可：

```bash
ros2 launch fast_livo luxi_device_cu013_mid360_preview.launch.py \
  external_trigger_enabled:=true \
  trigger_source:=Line0 \
  trigger_activation:=RisingEdge
```

如果需要临时关闭外触发，启动时加入：

```bash
ros2 launch fast_livo luxi_device_cu013_mid360_preview.launch.py external_trigger_enabled:=false
```

## STM32 同步模式

当 `MV-CU013-A0UC` 的 `Line0` 接 STM32 10Hz PWM，MID360 接 STM32 1Hz PPS 和时间戳输入时，建议用下面方式启动设备：

```bash
ros2 launch fast_livo luxi_device_cu013_mid360_preview.launch.py \
  mid360_xfer_format:=1 \
  external_trigger_enabled:=true \
  trigger_source:=Line0 \
  trigger_activation:=RisingEdge \
  use_rviz:=false
```

预期现象：

```bash
ros2 topic hz /hikrobot/cu013/rgb_img
# 应接近 10 Hz

ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
```

MID360 驱动日志中应出现 `livox time_type: 1` 或 `livox time_type: 2`。其中 `0` 表示未同步，`1` 表示 gPTP/PTP，`2` 表示 GPS/PPS 同步。FAST-LIVO2 使用前应确认不是 `0`。

## 常用开关

只看相机：

```bash
ros2 launch fast_livo luxi_device_cu013_mid360_preview.launch.py use_lidar:=false
```

只看雷达：

```bash
ros2 launch fast_livo luxi_device_cu013_mid360_preview.launch.py use_camera:=false
```

不启动 RViz：

```bash
ros2 launch fast_livo luxi_device_cu013_mid360_preview.launch.py use_rviz:=false
```
