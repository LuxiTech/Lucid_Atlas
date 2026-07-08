# go2_field_bringup

实机 GO2 的分层启动入口：

- `hw_sensors.launch.py`：Phase A 链路打通（驱动/雷达/RViz）
- `hw_localization.launch.py`：传感器 + AMCL 定位
- `hw_nav2.launch.py`：传感器 + Nav2 定位导航
- `hw_full.launch.py`：当前等价于 `hw_nav2`，用于后续扩展

## 遥控器优先（本阶段默认）

为避免干扰官方遥控器，`hw_sensors.launch.py` 里默认关闭了高层运动桥接：

- `enable_cmd_vel_bridge:=false`（默认）

后续做导航再打开（`hw_nav2`/`hw_full` 现在也默认关闭，防止误接管）：

```bash
ros2 launch go2_field_bringup hw_nav2.launch.py enable_cmd_vel_bridge:=true
```

若遥控器仍然无响应，先检查是否有节点在发官方运动请求：

```bash
ros2 topic info /api/sport/request -v
ros2 node list | grep -Ei 'cmd_vel_bridge|nav2|teleop|sport'
```

## 当前适配原则（已落实）

- 运动控制：使用官方高层接口 `/api/sport/request`
- 状态获取：使用官方高层状态 `/lf/sportmodestate`（可切 `/sportmodestate`）
- RViz：仅用于可视化（`robot_state_publisher + TF + PointCloud`）

## 常见显示问题（RobotModel 红色）

如果 RViz 显示 `No transform from [FL_...]` 这类错误，通常是没有 `joint_states`。  
当前默认已启用 `lowstate -> joint_states` 桥接（`enable_joint_state_bridge:=true`）。

快速检查：

```bash
ros2 topic hz /lowstate
ros2 topic hz /joint_states
```

如果你的低频状态不是 `/lowstate`，可改参数：

```bash
ros2 launch go2_field_bringup hw_sensors.launch.py \
  rviz:=true \
  lowstate_topic:=/lf/lowstate
```

## 最小链路打通

```bash
source /root/ws/unitree_ros2/setup.sh
source /root/ws/unitree_ros2/luxitech/go2/go2_robot/install/setup.bash
source /root/ws/unitree_ros2/luxitech/go2/go2_robot2/install/setup.bash

ros2 launch go2_field_bringup hw_sensors.launch.py rviz:=true
```

上述命令是“遥控器优先”模式：  
可以同时看到 RViz 的模型/TF/雷达（点云），并且不接管机器狗运动控制。

如果你想要“红色2D扫描线/点”（类似仿真里的 LaserScan 效果），请启用：

```bash
ros2 launch go2_field_bringup hw_sensors.launch.py \
  rviz:=true \
  rviz_mode:=scan2d \
  enable_scan_bridge:=true
```

若 `/scan` 有数据但 RViz 没点，优先检查 QoS（`/scan` 是 `Best Effort`）。  
本项目的 `scan2d` 配置已固定为 `Best Effort`，并默认在 `base_link` 坐标系生成扫描，保证前向与机身前向一致。

若你希望优先看“周围障碍物范围”（而不是仅近距离细节），可直接用：

```bash
ros2 launch go2_field_bringup hw_sensors.launch.py \
  rviz:=true \
  rviz_mode:=scan2d \
  enable_scan_bridge:=true
```

说明：
- GO2 前置雷达本身是前向视场，不是 360° 机械旋转雷达。
- 当前默认是“单水平切片”模式：`target_frame=odom`，`min_height=-0.03`，`max_height=0.03`，即只看 `Z=0` 附近的一层。
- RViz 的 `Decay Time` 默认是 `0`，只显示当前帧，不再叠成面。
- 默认显示的是过滤后的边界话题 `/scan_obstacles`（不是原始 `/scan`），会去掉近场机身杂点和离散飞点。

如需进一步“只看墙和障碍边界”，通过滑条面板只调“边缘明显程度”：

```bash
ros2 run go2_field_bringup scan_tuning_gui.py
```

## 可视化滑条调参（实时生效 + 自动保存）

启动链路后，打开滑条面板：

```bash
ros2 run go2_field_bringup scan_tuning_gui.py
```

或者在 launch 里直接打开：

```bash
ros2 launch go2_field_bringup hw_sensors.launch.py \
  rviz:=true rviz_mode:=scan2d enable_scan_bridge:=true \
  enable_scan_tuning_gui:=true
```

调参面板会实时下发到：
- `/scan_boundary_filter`

并自动保存到：
- `go2_field_config/config/scan_tuning.yaml`（可通过环境变量 `GO2_SCAN_TUNING_FILE` 覆盖）

注意：
- 面板现在只有一个主滑条 `Edge Clarity`，只调边缘明显程度。
- 下次重启 `hw_sensors.launch.py` 会自动读取 `scan_tuning.yaml` 的结果。

如系统提示缺少 Tk 图形库，可安装：

```bash
apt-get install -y python3-tk
```

如需切回官方 `go2_rviz` 默认界面：

```bash
ros2 launch go2_field_bringup hw_sensors.launch.py \
  rviz:=true \
  rviz_mode:=official
```

如果高层状态话题用的是 `/sportmodestate`，可切换：

```bash
ros2 launch go2_field_bringup hw_sensors.launch.py \
  rviz:=true \
  sport_state_topic:=/sportmodestate
```

## 键盘控制（高层接口）

当你需要不用遥控器、改成键盘控制时，启用：

```bash
ros2 launch go2_field_bringup hw_sensors.launch.py \
  rviz:=true \
  enable_keyboard_teleop:=true
```

控制键位：

- `w/s`：前进/后退
- `a/d`：左移/右移
- `←/→` 或 `q/e`：左转/右转
- `space` 或 `k`：急停

可调速度参数：

```bash
ros2 launch go2_field_bringup hw_sensors.launch.py \
  rviz:=true \
  enable_keyboard_teleop:=true \
  keyboard_linear_speed:=0.6 \
  keyboard_lateral_speed:=0.6 \
  keyboard_angular_speed:=1.0
```

## 前置摄像头在 RViz 显示（可选）

可以显示，支持两种方式：

- 已经有标准图像话题（`sensor_msgs/Image`）
- 使用本工程内置桥接：`unitree_go/msg/Go2FrontVideoData -> sensor_msgs/Image`

1. 检查图像话题：
```bash
ros2 topic list | grep -Ei 'front|camera|image|video'
```

2. 启动 RViz 后添加 `Image` Display，Topic 选实际话题（常见如 `/camera/color/image_raw`）。

3. 若只有压缩图（`.../compressed`），先转成 raw 再在 RViz 里选 raw 话题。

启用内置前置视频桥接（默认关闭）：

```bash
ros2 launch go2_field_bringup hw_sensors.launch.py \
  rviz:=true \
  enable_front_video_bridge:=true \
  front_video_topic:=/front_video_data \
  front_image_topic:=/front_camera/image_raw
```

然后在 RViz 添加 `Image` Display，Topic 选 `/front_camera/image_raw`。

注意：若日志提示 `failed to decode Go2FrontVideoData`，说明当前前置视频 payload 不是 JPEG，需要按实际编码格式再补解码器。

快速诊断：

```bash
ros2 topic list | grep -Ei 'front|camera|image|video'
ros2 topic type /front_video_data
ros2 topic hz /front_video_data
ros2 topic echo /front_video_data --once
```

## 常见报错：`package 'go2_field_bringup' not found`

这是工作空间未编译或未重新 source 的问题。执行：

```bash
cd /root/ws/unitree_ros2/luxitech/go2/go2_robot2
source /opt/ros/humble/setup.bash
colcon build --packages-select go2_field_bringup go2_field_config --symlink-install
source install/setup.bash
ros2 pkg list | grep -E '^go2_field_bringup$'
```

## 无实机时的链路联调（可选）

当 `/utlidar/robot_pose` 尚未提供时，`odom->base_link` 不会形成，Nav2 会卡在等待 TF。  
可临时启用假里程计（仅联调用）：

```bash
ros2 launch go2_field_bringup hw_nav2.launch.py \
  rviz:=false \
  enable_scan_bridge:=false \
  enable_fake_odom:=true \
  enable_fake_map_to_odom:=true
```

实机上必须关闭 `enable_fake_odom` 和 `enable_fake_map_to_odom`。

## 关键验证命令

```bash
ros2 topic hz /lf/sportmodestate
ros2 topic hz /odom
ros2 topic hz /scan
ros2 topic hz /pointcloud
ros2 topic hz /joint_states
ros2 topic list | grep -Ei 'front|camera|image|video'
ros2 topic echo /odom --once
```
