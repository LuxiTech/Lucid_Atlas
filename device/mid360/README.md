# MID360 工作区使用说明

本文档对应当前目录：

```bash
/root/ws/MID360
```

目标是说明这个工作区里的仓库结构、系统要求、编译命令、运行命令，以及如何验证 MID360 原始点云是否真的已经发布出来。

## 1. 当前目录结构

当前工作区已经整理为一个单独的自定义仓库，主要内容如下：

```bash
/root/ws/MID360
├── FAST_LIO_ROS2
├── Livox-SDK2
├── docs
└── src
    └── livox_ros_driver2
```

说明：

- `FAST_LIO_ROS2`：FAST-LIO 的 ROS2 版本代码
- `Livox-SDK2`：Livox 官方 SDK2
- `src/livox_ros_driver2`：Livox ROS2 驱动
- `docs`：本地说明文档

工作区构建产物统一放在：

- `/root/ws/MID360/build`
- `/root/ws/MID360/install`
- `/root/ws/MID360/log`

这些目录已经在顶层 `.gitignore` 中忽略，不会再作为源码提交。

## 2. 系统要求

当前这套工作区按下面环境使用：

- Ubuntu 20.04
- ROS2 Foxy
- `colcon`
- `cmake`
- 已安装 `Livox-SDK2`

当前容器里默认使用的 ROS 环境已经改成了：

```bash
/opt/ros/foxy
```

并且 `root` 用户的 `~/.bashrc` 已经默认配置了：

- `source /opt/ros/foxy/setup.bash`
- `source /root/ws/MID360/install/setup.bash`
- `DISPLAY=:1`
- `unset XAUTHORITY`
- `LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH`

所以后面很多命令只需要：

```bash
source /root/.bashrc
```

## 3. 当前 MID360 网络配置

当前驱动配置文件：

```bash
/root/ws/MID360/src/livox_ros_driver2/config/MID360_config.json
```

当前配置值是：

- 主机 IP：`192.168.1.50`
- 雷达 IP：`192.168.1.147`

如果以后更换网卡 IP 或更换雷达模块，需要优先修改这个文件里的：

- `host_net_info.*_ip`
- `lidar_configs[0].ip`

## 4. 编译命令

### 4.1 编译 Livox-SDK2

如果需要重新编译 SDK：

```bash
cd /root/ws/MID360/Livox-SDK2
mkdir -p build
cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig
```

### 4.2 编译 livox_ros_driver2 和 livox_sdk2

当前推荐的工作区编译命令是：

```bash
cd /root/ws/MID360
unset ROS_DISTRO ROS_VERSION ROS_PACKAGE_PATH CMAKE_PREFIX_PATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH
source /opt/ros/foxy/setup.bash
colcon build --packages-select livox_sdk2 livox_ros_driver2 --cmake-args -DROS_EDITION=ROS2
```

编译完成后加载环境：

```bash
source /root/ws/MID360/install/setup.bash
```

### 4.3 重新加载默认环境

因为 `~/.bashrc` 已经做了默认配置，所以通常只需要：

```bash
source /root/.bashrc
```

## 5. 运行命令

### 5.1 方案 A：只看原始雷达点云

最直接的命令：

```bash
source /root/.bashrc
mid360-rviz
```

它等价于：

```bash
ros2 launch /root/ws/MID360/src/livox_ros_driver2/launch_ROS2/rviz_MID360_launch.py
```

这个 launch 会：

- 启动 `livox_ros_driver2`
- 发布 `sensor_msgs/msg/PointCloud2`
- 自动打开 RViz

### 5.2 方案 B：只启动驱动，不开 RViz

```bash
source /root/.bashrc
mid360-msg
```

它等价于：

```bash
ros2 launch /root/ws/MID360/src/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py
```

这个 launch 会：

- 启动 `livox_ros_driver2`
- 发布 Livox 自定义消息 `livox_ros_driver2/msg/CustomMsg`
- 发布 `/livox/imu`

它更适合给 FAST-LIO 使用。

## 6. 如何确认点云真的在发布

不要只看 RViz 画面。最稳妥的方式是先看终端话题状态。

### 6.1 查看话题列表

驱动运行时，在另一个终端执行：

```bash
source /root/.bashrc
ros2 topic list
```

如果是原始点云模式，应该至少能看到：

```bash
/livox/lidar
/livox/imu
```

### 6.2 查看话题类型和发布者

```bash
source /root/.bashrc
ros2 topic info /livox/lidar -v
```

原始点云模式下，正常结果应类似：

```bash
Type: sensor_msgs/msg/PointCloud2
Publisher count: 1
```

如果 `Publisher count: 0`，说明驱动还没有真正发布点云。

### 6.3 直接打印一帧点云消息

```bash
source /root/.bashrc
timeout 5 ros2 topic echo /livox/lidar | sed -n '1,40p'
```

正常会看到类似内容：

```yaml
header:
  frame_id: livox_frame
height: 1
width: 20064
fields:
- name: x
- name: y
- name: z
- name: intensity
```

其中：

- `frame_id: livox_frame` 说明坐标系正常
- `width` 大于 0 说明这帧里确实有点

### 6.4 查看点云频率

```bash
source /root/.bashrc
timeout 6 ros2 topic hz /livox/lidar
```

如果驱动正在稳定发点云，会输出一段频率统计。

## 7. RViz 里推荐的显示设置

如果 RViz 已经打开，但点云不明显，建议在左侧 `PointCloud2` 设置里检查：

- `Topic`: `/livox/lidar`
- `Style`: `Flat Squares`
- `Size (m)`: `0.05`
- `Color Transformer`: `Intensity`
- `Fixed Frame`: `livox_frame`

如果画面还是看不清：

1. 点击右侧 `Views` 面板的 `Zero`
2. 鼠标滚轮缩放
3. 左键旋转视角
4. 中键平移

## 8. 当前已经验证通过的命令

下面这条链路已经实际验证通过：

### 启动原始点云 RViz

```bash
source /root/.bashrc
mid360-rviz
```

### 在另一个终端验证点云

```bash
source /root/.bashrc
ros2 topic info /livox/lidar -v
timeout 5 ros2 topic echo /livox/lidar | sed -n '1,40p'
```

实测结果包含：

```text
Type: sensor_msgs/msg/PointCloud2
Publisher count: 1
```

以及：

```text
frame_id: livox_frame
width: 20064
```

说明当前新雷达 `192.168.1.147` 的原始点云已经可以正常发布。

## 9. FAST-LIO 使用说明

### 9.1 先决条件

FAST-LIO 依赖 Livox 自定义消息，所以要先启动：

```bash
source /root/.bashrc
mid360-msg
```

也就是：

```bash
ros2 launch /root/ws/MID360/src/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py
```

### 9.2 当前状态

`FAST_LIO_ROS2` 目录已经并入当前工作区，但它还没有完全整理成“当前环境下一条命令稳定编译通过”的状态。

因此目前推荐流程是：

1. 先用 `livox_ros_driver2` 验证原始点云
2. 再单独处理 `FAST_LIO_ROS2` 的编译与适配

如果后续要继续推进 FAST-LIO，请从这里开始：

```bash
/root/ws/MID360/FAST_LIO_ROS2
```

## 10. Git 使用说明

当前 `/root/ws/MID360` 已按单仓库方式整理：

- `FAST_LIO_ROS2`
- `Livox-SDK2`
- `src/livox_ros_driver2`

都已经作为普通目录纳入顶层仓库管理，而不是继续保留各自独立的 `.git`。

同时，以下目录不会再被 Git 跟踪：

- `/root/ws/MID360/build`
- `/root/ws/MID360/install`
- `/root/ws/MID360/log`
- 子目录里的构建输出 `build/`

可以用下面命令检查当前状态：

```bash
cd /root/ws/MID360
git status
```

## 11. 常用命令速查

### 编译

```bash
cd /root/ws/MID360
unset ROS_DISTRO ROS_VERSION ROS_PACKAGE_PATH CMAKE_PREFIX_PATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH
source /opt/ros/foxy/setup.bash
colcon build --packages-select livox_sdk2 livox_ros_driver2 --cmake-args -DROS_EDITION=ROS2
```

### 启动原始点云 RViz

```bash
source /root/.bashrc
mid360-rviz
```

### 启动自定义消息驱动

```bash
source /root/.bashrc
mid360-msg
```

### 检查点云话题

```bash
source /root/.bashrc
ros2 topic info /livox/lidar -v
timeout 5 ros2 topic echo /livox/lidar | sed -n '1,40p'
```

