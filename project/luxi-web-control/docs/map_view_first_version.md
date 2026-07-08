# 雷达建图网页查看第一版目标

## 目标

在现有 `luxi_web_control` 网页控制程序中增加“雷达地图”查看能力。第一版不做真实导航闭环、不做地图保存，只验证手机或本机浏览器可以稳定访问网页，并稳定查看当前 ROS2 中的地图、实时点云和机器人位姿状态。

## 第一版范围

必须实现：

- 网页中增加“雷达地图”区域。
- 后端订阅 `/map`，用于显示 2D 栅格地图。
- 后端订阅 `/luxi_localization/map_cloud_floor`，用于显示离线地图投影点云。
- 后端订阅 `/luxi_localization/aligned_cloud_floor`，用于显示实时雷达投影点云。
- 后端订阅 `/localization_2d`，用于显示机器人当前位置和朝向。
- 网页通过 `http://设备WiFi_IP:8082/api/map/snapshot` 拉取地图快照。
- 数据源不存在时，网页显示“等待数据”，服务不崩溃、不阻塞。

暂不实现：

- 完整 3D OctoMap 体素地图。
- 地图保存、地图加载、建图开关。
- 导航路径规划和导航执行。
- WebSocket 二进制推流。

## 当前推荐数据源

当前 `luxi-localization` 中已经具备适合网页显示的数据：

```text
/map
/luxi_localization/map_cloud_floor
/luxi_localization/aligned_cloud_floor
/localization_2d
```

第一版使用投影到地面平面的点云，而不是直接发送完整 3D 点云。这样手机端显示压力更低，WiFi 传输也更稳定。

## 后端接口

复用现有 `web_api_node` 的 HTTP API 端口，默认是 `8082`。

### GET /api/map/status

返回地图数据状态：

```json
{
  "ok": true,
  "has_grid": true,
  "has_static_cloud": true,
  "has_live_cloud": true,
  "has_pose": true,
  "grid_topic": "/map",
  "static_cloud_topic": "/luxi_localization/map_cloud_floor",
  "live_cloud_topic": "/luxi_localization/aligned_cloud_floor",
  "pose_topic": "/localization_2d"
}
```

### GET /api/map/snapshot

返回当前网页可绘制的数据快照：

```json
{
  "ok": true,
  "grid": {
    "available": true,
    "width": 1000,
    "height": 1000,
    "resolution": 0.05,
    "origin": {"x": -25.0, "y": -25.0, "yaw": 0.0},
    "data": [0, 0, 100, -1]
  },
  "static_cloud": {
    "available": true,
    "frame_id": "map",
    "points": [[1.0, 2.0, 0.0]]
  },
  "live_cloud": {
    "available": true,
    "frame_id": "map",
    "points": [[1.1, 2.0, 0.0]]
  },
  "pose": {
    "available": true,
    "x": 0.0,
    "y": 0.0,
    "yaw": 0.0
  }
}
```

## 稳定性策略

- 点云在 C++ 后端限点数，避免浏览器和 WiFi 被完整点云压垮。
- `/api/map/snapshot` 始终返回合法 JSON；没有数据时返回 `available:false`。
- 前端轮询失败时不清空页面，只更新状态文本。
- 前端支持图层开关：栅格地图、静态点云、实时点云、机器人位姿。
- 第一版使用 2D Canvas 绘制，后续再升级 Three.js/WebSocket。

## 默认参数

```yaml
web_api_node:
  ros__parameters:
    map_topic: /map
    static_cloud_topic: /luxi_localization/map_cloud_floor
    live_cloud_topic: /luxi_localization/aligned_cloud_floor
    pose_topic: /localization_2d
    max_static_cloud_points: 60000
    max_live_cloud_points: 12000
    max_grid_cells: 250000
```

## 验收方式

本地至少验证：

- `colcon build --packages-select luxi_web_control` 构建通过。
- `ros2 launch luxi_web_control web_control.launch.py bind_address:=127.0.0.1` 可以稳定拉起服务。
- `curl http://127.0.0.1:8082/api/map/status` 返回合法 JSON。
- `curl http://127.0.0.1:8082/api/map/snapshot` 返回合法 JSON。
- 本机访问 `http://127.0.0.1:8080` 页面正常加载。

实机验证：

- 启动定位或建图：

```bash
ros2 launch luxi_localization bringup_localization.launch.py
```

- 启动网页控制：

```bash
ros2 launch luxi_web_control web_control.launch.py
```

- 手机连接同一 WiFi 后访问：

```text
http://设备WiFi_IP:8080
```

