# FAST-LIO2 MID360 Commands

本文档记录本工程在本机使用 Livox MID360 运行 FAST-LIO2、保存地图、使用已保存地图，以及基于三维点云地图做规划/导航的建议路线。

## 1. 启动 MID360 驱动

本机当前 MID360 驱动工作区在 `/root/ws/MID360`，Livox 配置中主机 IP 为 `192.168.1.50`，雷达 IP 为 `192.168.1.147`。

先确认网络：

```bash
ping 192.168.1.147
```

启动 Livox ROS2 驱动：

```bash
source /opt/ros/foxy/setup.bash
source /root/ws/MID360/install/setup.bash

ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

另开终端检查话题：

```bash
source /opt/ros/foxy/setup.bash
source /root/ws/MID360/install/setup.bash

ros2 topic list | grep livox
ros2 topic hz /livox/imu
ros2 topic hz /livox/lidar
```

正常应看到：

```text
/livox/imu
/livox/lidar
```

其中 `/livox/lidar` 是 `livox_ros_driver2/msg/CustomMsg`，这是 FAST-LIO2 对 Livox 雷达进行点级时间去畸变所需要的消息格式。

## 2. 启动 FAST-LIO2 建图

如果使用 `/root/ws/MID360` 中已经构建好的 FAST-LIO2：

```bash
source /opt/ros/foxy/setup.bash
source /root/ws/MID360/install/setup.bash
source /root/ws/MID360/install/fast_lio/share/fast_lio/local_setup.bash

export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libusb-1.0.so.0

ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml
```

如果不需要打开 RViz：

```bash
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml rviz:=false
```

`LD_PRELOAD` 的原因：当前机器环境里 `/opt/MVS/lib/64` 会优先加载海康 MVS 自带的旧 `libusb`，PCL 运行时可能报：

```text
undefined symbol: libusb_set_option
```

预加载系统版 `libusb` 可以避免这个冲突。

启动后检查 FAST-LIO2 输出：

```bash
ros2 topic list | grep -E 'Odometry|path|cloud_registered|Laser_map'
ros2 run tf2_ros tf2_echo camera_init body
```

常用输出：

```text
/Odometry
/path
/cloud_registered
/cloud_registered_body
/Laser_map
```

坐标系：

```text
camera_init -> body
```

其中 `camera_init` 可理解为本次建图启动时的世界/地图原点，`body` 是 LiDAR-IMU 机体系。

## 3. 保存点云地图

当前 `config/mid360.yaml` 已经开启 PCD 保存，并将默认保存路径设为：

```yaml
map_file_path: "/root/ws/catkin_ws/src/FAST_LIO_ROS2/maps/fast_lio_map.pcd"

publish:
  map_en: true

pcd_save:
  pcd_save_en: true
  interval: -1
```

先创建地图目录：

```bash
mkdir -p /root/ws/catkin_ws/src/FAST_LIO_ROS2/maps
```

使用默认文件名保存地图：

```bash
ros2 service call /map_save std_srvs/srv/Trigger {}
```

成功时会返回类似：

```text
success: true
message: Map saved to /root/ws/catkin_ws/src/FAST_LIO_ROS2/maps/fast_lio_map.pcd
```

如果希望命令中指定地图名字，使用 `/map_save_with_name`：

```bash
ros2 service call /map_save_with_name fast_lio/srv/SaveMap "{map_name: room_001}"
```

这个命令会保存为：

```text
/root/ws/catkin_ws/src/FAST_LIO_ROS2/maps/room_001.pcd
```

如果 `map_name` 没有 `.pcd` 后缀，程序会自动补上。地图名中的 `/`、空格等不适合作为文件名的字符会被替换为 `_`。

也可以直接传带 `.pcd` 后缀的名字：

```bash
ros2 service call /map_save_with_name fast_lio/srv/SaveMap "{map_name: lab_map.pcd}"
```

注意：

- `/map_save` 使用 `map_file_path` 保存，当前默认是绝对路径 `/root/ws/catkin_ws/src/FAST_LIO_ROS2/maps/fast_lio_map.pcd`。
- `/map_save_with_name` 使用请求里的 `map_name` 保存，固定保存到 `/root/ws/catkin_ws/src/FAST_LIO_ROS2/maps`。
- 当前 `/map_save` 保存的是 `/Laser_map` 里累计的世界系点云，依赖 `publish.map_en: true`。
- 本工程源码中退出时自动保存 `PCD/scans.pcd` 的逻辑依赖 `pcl_wait_save`，但当前逐帧累计到 `pcl_wait_save` 的代码块是注释状态。因此推荐使用 `/map_save` 服务保存地图。

如果修改的是源码目录里的配置：

```bash
/root/ws/catkin_ws/src/FAST_LIO_ROS2/config/mid360.yaml
```

运行安装版包时需要重新构建，或者启动时显式指定源码配置路径：

```bash
ros2 launch fast_lio mapping.launch.py \
  config_path:=/root/ws/catkin_ws/src/FAST_LIO_ROS2/config \
  config_file:=mid360.yaml
```

## 4. 查看和处理保存的 PCD 地图

如果系统没有 `pcl_viewer`，先安装 PCL 工具：

```bash
apt-get install -y pcl-tools
```

使用 PCL 查看：

```bash
LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libusb-1.0.so.0 \
  pcl_viewer /root/ws/catkin_ws/src/FAST_LIO_ROS2/maps/fast_lio_map.pcd
```

查看指定地图，例如 `room_001.pcd`：

```bash
LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libusb-1.0.so.0 \
  pcl_viewer /root/ws/catkin_ws/src/FAST_LIO_ROS2/maps/room_001.pcd
```

查看点云基本信息：

```bash
pcl_pcd2ply /root/ws/catkin_ws/src/FAST_LIO_ROS2/maps/fast_lio_map.pcd /root/ws/catkin_ws/src/FAST_LIO_ROS2/maps/fast_lio_map.ply
```

如果地图太大，建议先做体素降采样。可使用 PCL 工具或自己写一个小节点/脚本做 voxel grid，例如：

```text
leaf_size = 0.05 ~ 0.20 m
```

常见后处理目标：

- 降采样：减小 PCD 体积，提升定位和规划速度。
- 裁剪高度：去掉天花板、地面下方噪声或远处离群点。
- 去离群点：移除孤立点。
- 转换格式：PCD 转 PLY、OctoMap、occupancy grid、grid map 等。

## 5. 已保存地图怎么使用

FAST-LIO2 原始建图节点主要负责实时里程计和增量建图，并不等于完整的“加载旧地图后全局定位”系统。保存的 PCD 地图通常有三类用途。

### 5.1 离线查看和测量

直接用 `pcl_viewer`、CloudCompare、RViz 自定义点云发布节点查看。

### 5.2 作为定位地图

如果机器人下次启动还想使用同一张地图导航，需要增加重定位/定位模块。常用方案：

- NDT localization：把当前 LiDAR 点云和已保存 PCD 地图做 NDT 匹配。
- ICP/GICP localization：把当前局部点云和 PCD 地图做 ICP 匹配。
- FAST_LIO_LOCALIZATION：使用 FAST-LIO 系列的 localization 扩展方案。
- 2D AMCL：把 PCD 投影成 2D 栅格地图后，用 AMCL 做平面定位。

定位模块应输出：

```text
map -> odom
```

FAST-LIO2 实时输出：

```text
odom 或 camera_init -> base_link 或 body
```

最终导航需要形成标准 TF：

```text
map -> odom -> base_link
```

在本工程中可先近似对应：

```text
camera_init ~= odom
body ~= base_link
```

但这只适合单次启动、短时运行。跨启动复用旧地图时，必须有重定位模块来确定 `map -> odom`。

### 5.3 转成规划地图

路径规划不建议直接在原始 PCD 上做。更常见做法是把 PCD 转成适合规划的数据结构：

- 2D 占据栅格：适合地面移动机器人接 Nav2。
- OctoMap：适合三维占据表达。
- Voxel grid：适合局部避障和三维碰撞检测。
- ESDF/TSDF：适合无人机或三维轨迹优化。
- Elevation map：适合足式机器人或越野车。

## 6. 使用三维雷达地图做路径规划和导航的实现思路

FAST-LIO2 提供的是：

```text
LiDAR + IMU -> 实时位姿 /Odometry + 点云地图 /Laser_map + 注册点云 /cloud_registered
```

完整导航系统还需要：

```text
地图表示 + 定位 + 全局规划 + 局部规划/避障 + 控制器
```

推荐按机器人类型选择路线。

### 6.1 地面移动机器人，优先走 2D Nav2

这是最容易落地的方案。

流程：

```text
FAST-LIO2 建图
  -> 保存 PCD
  -> PCD 按高度裁剪并投影成 2D occupancy grid
  -> Nav2 map_server 加载 2D 地图
  -> 定位模块输出 map -> odom
  -> Nav2 planner/controller 输出 /cmd_vel
```

局部避障输入可以来自：

```text
/cloud_registered_body -> pointcloud_to_laserscan -> /scan
```

或者直接给 Nav2 obstacle/voxel layer 使用 PointCloud2。

优点：

- 工程成熟，Nav2 生态完整。
- 适合轮式底盘。
- 调参和可视化方便。

限制：

- 主要处理平面导航。
- 三维结构会被压到 2D，占据高度需要仔细设置。

### 6.2 地面机器人，保留 3D 障碍物做局部避障

如果环境有桌椅、坡道、悬空障碍等，建议使用：

```text
2D 全局规划 + 3D 局部避障
```

实现方式：

- PCD 投影成 2D map，供全局规划。
- 实时 `/cloud_registered_body` 构建局部 voxel costmap。
- 局部规划器检查机器人 footprint 与 voxel 障碍物碰撞。
- 控制器输出速度命令。

这样既能保留 Nav2 的稳定全局规划，又能利用 MID360 的三维感知能力。

### 6.3 无人机或需要真正三维运动的机器人

如果机器人需要在三维空间运动，不能只用 Nav2 2D。建议路线：

```text
FAST-LIO2 位姿
  -> PCD/实时点云
  -> OctoMap 或 voxel grid
  -> ESDF
  -> A* / RRT* / kinodynamic A* / trajectory optimization
  -> 轨迹跟踪控制器
```

常见模块：

- 地图：OctoMap、Voxblox、nvblox、FIESTA、grid_map。
- 全局规划：A*、RRT*、BIT*。
- 动力学规划：kinodynamic A*、minimum snap、MPC。
- 局部避障：ESDF gradient、velocity obstacle、MPC collision constraint。

关键要求：

- 必须有稳定的 `map -> body` 位姿。
- 地图需要不断清理动态障碍或做局部更新。
- 规划地图要膨胀机器人半径，保留安全距离。
- 控制器需要考虑机器人动力学约束，而不是只输出几何路径。

## 7. 推荐落地路线

如果当前目标是 MID360 小车导航，建议按下面顺序做：

1. 跑通 FAST-LIO2，确认 `/Odometry`、`/Laser_map`、`/cloud_registered_body` 正常。
2. 使用 `/map_save` 保存 PCD。
3. 将 PCD 裁剪高度并投影成 2D occupancy grid。
4. 接 Nav2，先完成静态地图导航。
5. 增加 `pointcloud_to_laserscan` 或 voxel layer，把实时点云用于局部避障。
6. 增加重定位模块，解决重启后复用旧地图的问题。
7. 如果 2D 不够，再升级到 OctoMap/ESDF 的三维规划链路。

最小可运行系统：

```text
MID360 driver
  -> FAST-LIO2
  -> /Odometry + /cloud_registered_body
  -> Nav2 + costmap
  -> /cmd_vel
```

可复用旧地图的系统：

```text
saved PCD map
  -> localization module
  -> map -> odom

FAST-LIO2
  -> odom -> base_link

Nav2 / 3D planner
  -> /cmd_vel or trajectory
```
