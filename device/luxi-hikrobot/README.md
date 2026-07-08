# Luxitech Hikrobot Camera

本仓库用于整理海康机器人 Hikrobot/MVS 工业相机在 Linux/ROS2 环境下的二次开发代码，当前重点适配：

- `MV-CH120-60UC`：保留原始示例代码
- `MV-CU013-A0UC`：已完成 ARM64 SDK 适配、OpenCV 示例、ROS2 图像发布 sample

仓库只提交 BSP、示例工程、ROS2 节点和文档。海康官方 SDK 体积较大，不提交到 Git，请按本文说明单独下载并放到本地指定目录。

## SDK 下载与存放

### 官方下载入口

从海康机器人机器视觉下载中心下载 Linux 版本 MVS/SDK：

```text
https://www.hikrobotics.com/cn/machinevision/service/download/?fileType=2&id=49&module=0&operateSystem=Linux&typeId=40
```

也可以从下载中心首页按条件筛选：

```text
https://www.hikrobotics.com/cn/machinevision/service/download/?module=0
```

筛选建议：

- 类型：软件 / SDK / MVS
- 系统：Linux
- 架构：ARM64 / aarch64
- 相机：USB3 Vision 工业相机

### 本地存放路径

当前设备上使用的 SDK 包放在：

```text
/home/nvidia/project/luxi-atlas/device/luxi-hikrobot/SDK/MvCamCtrlSDK_STD_V4.8.0_260512.zip
```

解压后的 Runtime 路径：

```text
/home/nvidia/project/luxi-atlas/device/luxi-hikrobot/SDK/extracted/MvCamCtrlSDK_Runtime-4.8.0_aarch64_20260512
```

本仓库示例和 ROS2 节点使用的 SDK runtime 路径：

```text
/home/nvidia/project/luxi-atlas/device/luxi-hikrobot/SDK/runtime/MvCamCtrlSDK
```

运行前需要加载 SDK 环境变量：

```bash
source /home/nvidia/project/luxi-atlas/device/luxi-hikrobot/SDK/extracted/MvCamCtrlSDK_Runtime-4.8.0_aarch64_20260512/set_env_path.sh \
  /home/nvidia/project/luxi-atlas/device/luxi-hikrobot/SDK/runtime/MvCamCtrlSDK 4.8.0
```

注意：`SDK/` 已加入 `.gitignore`，不要把 SDK zip、解压目录、动态库提交到仓库。

## 环境依赖

推荐环境：

- Ubuntu 22.04 ARM64
- ROS2 Humble
- C++17
- CMake >= 3.16
- OpenCV 4
- 海康 MVS Runtime 4.8.0 ARM64

基础依赖：

```bash
sudo apt update
sudo apt install -y build-essential cmake pkg-config libopencv-dev
sudo apt install -y kmod udev iproute2 usbutils
sudo apt install -y ros-humble-rclcpp ros-humble-sensor-msgs ros-humble-image-transport ros-humble-rviz2
```

USB 相机需要配置 udev 权限。当前设备已按 `2bdf:0001` 配置过海康 USB 设备权限。

## 项目结构

```text
luxi-hikrobot/
├── bsp/
│   ├── include/luxitech/mvs/
│   └── src/
├── Project/
│   ├── MV-CH120-60UC/
│   │   ├── 01-mini_visualization/
│   │   └── 02-adj_visualization/
│   └── MV-CU013-A0UC/
│       ├── bsp/
│       ├── 01-mini_visualization/
│       └── 02-adj_visualization/
├── ROS2/
│   └── MV-CU013-A0UC/
│       └── ws/src/luxi_hikrobot_cu013_camera/
├── SDK/                  # 本地 SDK，Git 忽略
├── docs/
└── README.md
```

## MV-CU013-A0UC OpenCV 示例

### 01-mini_visualization

默认参数：

- `ExposureAuto=Off`
- `ExposureTime=25000`
- `GainAuto=Off`
- `Gain=15`

构建运行：

```bash
cd /home/nvidia/project/luxi-atlas/device/luxi-hikrobot/Project/MV-CU013-A0UC/01-mini_visualization
source /home/nvidia/project/luxi-atlas/device/luxi-hikrobot/SDK/extracted/MvCamCtrlSDK_Runtime-4.8.0_aarch64_20260512/set_env_path.sh \
  /home/nvidia/project/luxi-atlas/device/luxi-hikrobot/SDK/runtime/MvCamCtrlSDK 4.8.0

cmake -S . -B build-arm \
  -DMVS_SDK_PATH=/home/nvidia/project/luxi-atlas/device/luxi-hikrobot/SDK/runtime/MvCamCtrlSDK
cmake --build build-arm -j
./build-arm/01-mini_visualization
```

### 02-adj_visualization

支持 OpenCV 按键调参、终端命令调参和保存当前帧。

```bash
cd /home/nvidia/project/luxi-atlas/device/luxi-hikrobot/Project/MV-CU013-A0UC/02-adj_visualization
source /home/nvidia/project/luxi-atlas/device/luxi-hikrobot/SDK/extracted/MvCamCtrlSDK_Runtime-4.8.0_aarch64_20260512/set_env_path.sh \
  /home/nvidia/project/luxi-atlas/device/luxi-hikrobot/SDK/runtime/MvCamCtrlSDK 4.8.0

cmake -S . -B build-arm \
  -DMVS_SDK_PATH=/home/nvidia/project/luxi-atlas/device/luxi-hikrobot/SDK/runtime/MvCamCtrlSDK
cmake --build build-arm -j
./build-arm/02-adj_visualization
```

常用按键：

- `[` / `]`：减小 / 增大曝光
- `-` / `=`：减小 / 增大增益
- `a`：切换自动曝光
- `g`：切换自动增益
- `p`：切换像素格式
- `s`：保存当前帧
- `q`：退出

## ROS2 MV-CU013-A0UC Sample

ROS2 sample 位于：

```text
/home/nvidia/project/luxi-atlas/device/luxi-hikrobot/ROS2/MV-CU013-A0UC/ws/src/luxi_hikrobot_cu013_camera
```

发布话题：

- `/hikrobot/cu013/image_raw`
- `/hikrobot/cu013/rgb_img`
- `/hikrobot/cu013/camera_info`

构建：

```bash
cd /home/nvidia/project/luxi-atlas/device/luxi-hikrobot/ROS2/MV-CU013-A0UC/ws
source /opt/ros/humble/setup.bash
source /home/nvidia/project/luxi-atlas/device/luxi-hikrobot/SDK/extracted/MvCamCtrlSDK_Runtime-4.8.0_aarch64_20260512/set_env_path.sh \
  /home/nvidia/project/luxi-atlas/device/luxi-hikrobot/SDK/runtime/MvCamCtrlSDK 4.8.0

colcon build --symlink-install --cmake-args \
  -DMVS_SDK_PATH=/home/nvidia/project/luxi-atlas/device/luxi-hikrobot/SDK/runtime/MvCamCtrlSDK
```

运行并打开 RViz：

```bash
cd /home/nvidia/project/luxi-atlas/device/luxi-hikrobot/ROS2/MV-CU013-A0UC/ws
source /opt/ros/humble/setup.bash
source install/setup.bash
source /home/nvidia/project/luxi-atlas/device/luxi-hikrobot/SDK/extracted/MvCamCtrlSDK_Runtime-4.8.0_aarch64_20260512/set_env_path.sh \
  /home/nvidia/project/luxi-atlas/device/luxi-hikrobot/SDK/runtime/MvCamCtrlSDK 4.8.0

ros2 launch luxi_hikrobot_cu013_camera cu013.launch.py
```

如果 RViz 空白，手动选择 Image Display 的 topic：

```text
/hikrobot/cu013/rgb_img
```

Transport Hint 保持 `raw`。

检查图像流：

```bash
ros2 topic list | grep hikrobot
ros2 topic hz /hikrobot/cu013/rgb_img
```

## 外部 PWM 触发

ROS2 配置文件：

```text
ROS2/MV-CU013-A0UC/ws/src/luxi_hikrobot_cu013_camera/config/cu013.yaml
```

默认连续采集：

```yaml
external_trigger_enabled: false
trigger_source: "Line0"
trigger_activation: "RisingEdge"
```

使用外部 PWM 输入测试：

```bash
ros2 launch luxi_hikrobot_cu013_camera cu013.launch.py \
  external_trigger_enabled:=true \
  trigger_source:=Line0 \
  trigger_activation:=RisingEdge
```

外部触发开启后，图像发布频率跟随 PWM 触发频率；未接入 PWM 时抓帧超时或无图像输出是正常现象。

## 开发说明

- `bsp/` 保留通用 CH120 原始 BSP 风格。
- `Project/MV-CU013-A0UC/bsp/src/mvs_camera.cpp` 是 CU013 专用适配版本。
- `Project/MV-CH120-60UC/` 保留 CH120 原始示例，不再为 CU013 修改。
- `ROS2/MV-CU013-A0UC/` 是型号隔离的 ROS2 sample，后续其他型号可以并列新增目录。
- `SDK/`、`build/`、`install/`、`log/`、`MvSdkLog/` 均不应提交。
