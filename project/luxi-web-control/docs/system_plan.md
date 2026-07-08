# Luxi Web Control 最小实现方案

## 当前目标

本包先实现一个最小网页控制闭环：

- 手机访问机器人网页。
- 默认网络模式是同一 WiFi 局域网访问，不占用有线雷达接口，不主动创建热点。
- 网页只通过同一 WiFi 局域网内的 C++ HTTP API 访问 ROS2，不启用热点，不依赖 ROSBridge。
- 网页显示系统状态、控制状态、网络状态。
- 点击网页的“显示RGB”后，通过 C++ API 显示 `/hikrobot/cu013/rgb_img` 的最新 JPEG 画面。
- 网页发布演示速度、急停、目标点。
- C++ 后端接收并限幅，但默认不转发到真实 `/cmd_vel`。

这版用于验证同 WiFi 局域网、网页、C++ HTTP API、手机交互链路，不直接控制真实机器人。

## 当前包结构

```text
project/luxi-web-control/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── web_control.yaml
├── launch/
│   └── web_control.launch.py
├── src/
│   ├── command_guard_node.cpp
│   ├── demo_state_node.cpp
│   └── network_status_node.cpp
├── web/
│   ├── index.html
│   ├── main.js
│   ├── styles.css
│   └── vendor/roslib.min.js
└── docs/
    └── system_plan.md
```

## 节点职责

### network_status_node

发布：

```text
/web/network_status  std_msgs/String(JSON)
```

它会读取：

```bash
nmcli device status
ip -4 addr show dev <wifi_interface>
```

它只报告 WiFi 局域网访问地址，不检测或启动热点。`enP8p1s0` 保留给雷达，不作为网页服务绑定地址。

### command_guard_node

订阅：

```text
/web/cmd_vel    geometry_msgs/Twist
/web/estop      std_msgs/Bool
/web/goal_pose  geometry_msgs/PoseStamped
```

发布：

```text
/web/accepted_cmd_vel  geometry_msgs/Twist
/web/control_status    std_msgs/String(JSON)
```

默认参数：

```yaml
enable_passthrough: false
output_cmd_topic: /cmd_vel
```

也就是说，默认只做演示接收，不会控制底盘。后续确认底盘安全链路以后，才可以把 `enable_passthrough` 改为 `true`。

### demo_state_node

发布：

```text
/web/system_status  std_msgs/String(JSON)
```

用于给网页提供稳定心跳，显示当前 demo 模式和最近一次被保护节点接收的速度。

## 网页服务安排

launch 默认启动：

```text
HTTP 静态网页: 0.0.0.0:8080
C++ HTTP API:   WiFi_IP:8082
```

实际 launch 会自动读取 `wlP1p1s0` 的 IPv4，并把 HTTP 静态网页和 C++ HTTP API 都绑定到这个 WiFi IP。它不会绑定 `enP8p1s0` 的以太网 IP。

手机访问：

```text
http://机器人IP:8080
```

网页会通过同一局域网访问：

```text
http://机器人IP:8082/api/status
http://机器人IP:8082/api/cmd_vel
http://机器人IP:8082/api/estop
http://机器人IP:8082/api/goal_pose
http://机器人IP:8082/api/camera/latest.jpg
http://机器人IP:8082/api/camera/stream.mjpg
http://机器人IP:8082/api/camera/exposure
```

当前默认同 WiFi 模式会在 `/web/network_status` 中发布：

```text
same_wifi_url
api_url
wifi_ip
```

本机当前 `wlP1p1s0` 检测到的 IPv4 是 `10.42.0.149`，所以手机与设备处于同一个 WiFi 时访问：

```text
http://10.42.0.149:8080
```

## 网络策略

启动网页控制不创建无线接入点，不修改 DNS，不修改默认路由。

仅启动网页：

```bash
ros2 launch luxi_web_control web_control.launch.py
```

## 后续接入真实功能

### 点云

新增 C++ 节点：

```text
cloud_filter_node
  subscribe: /cloud_registered_body 或 /luxi_localization/aligned_cloud
  publish:   /web/pointcloud
```

处理策略：

- 限频到 3-10Hz。
- 体素降采样到 0.10-0.25m。
- 限制最大点数。
- 手机端用 Three.js 渲染。

### RGB 图像

已有相机话题：

```text
/hikrobot/cu013/rgb_img
```

当前由 C++ HTTP API 把最新 `sensor_msgs/Image` 编码为 JPEG，再由网页按需刷新显示。

### 导航

新增 C++ 节点：

```text
navigation_bridge_node
  subscribe: /web/goal_pose
  output: Nav2 action 或 jie_3d_nav 的 /start_point /goal_point /goal_pose
```

这样网页只负责交互，导航细节留在 ROS2 后端。

### 真正控制底盘

确认安全后再启用：

```yaml
command_guard_node:
  ros__parameters:
    enable_passthrough: true
    output_cmd_topic: /cmd_vel
```

上线前至少保留：

- 速度限幅。
- 指令超时停车。
- 急停。
- 导航/手动控制互斥。
- 低电量、定位丢失、雷达异常时禁止运动。
