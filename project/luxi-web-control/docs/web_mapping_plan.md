# Web Mapping Integration Plan

## 目标

在现有网页控制系统中集成建图流程，让用户可以在网页端：

- 进入建图模式。
- 使用网页方向键控制机器人移动。
- 实时查看 FAST-LIO 累计地图点云。
- 输入地图名称。
- 保存当前 PCD 地图。

第一版重点是稳定打通“网页控制 + 实时点云 + 命名保存 PCD”。不在第一版中做复杂的建图进程生命周期管理。

## 现有基础

当前工程已经具备以下能力：

```text
luxi_web_control
  - 网页方向键控制 /web/cmd_vel
  - command_guard_node 转发 /cmd_vel
  - web_api_node 提供 HTTP API
  - web_api_node 已能下采样并输出点云到网页

luxi-atlas-lio / FAST-LIO
  - 发布 /cloud_registered
  - 发布 /cloud_registered_body
  - 发布 /Laser_map
  - 提供 /map_save
  - 提供 /map_save_with_name
```

FAST-LIO 保存服务定义：

```text
string map_name
---
bool success
string message
string path
```

## 第一版边界

第一版不从网页启动或杀死 FAST-LIO 进程。原因：

- 当前总服务已经会拉起雷达、FAST-LIO、定位和网页控制。
- 进程级启动/停止涉及 launch 进程管理、传感器资源互斥和异常恢复，适合第二版实现。
- 第一版先保证网页能看到建图点云并能调用保存服务。

第一版的“开始建图”含义：

```text
进入建图会话
记录地图名称
显示 /Laser_map 累计地图点云
允许网页控制机器人移动
```

第一版的“停止建图”含义：

```text
退出建图会话
不停止 FAST-LIO 进程
不自动保存地图
```

第一版的“保存PCD”含义：

```text
调用 /map_save_with_name
由 FAST-LIO 将当前累计地图保存为指定名称
```

## 数据链路

```text
网页点击开始建图
  -> POST /api/mapping/start
  -> web_api_node 记录 mapping_active=true
  -> 网页打开建图点云图层

FAST-LIO
  -> /Laser_map
  -> web_api_node 下采样
  -> GET /api/map/snapshot
  -> 网页 Canvas 绘制建图点云

网页方向键
  -> POST /api/cmd_vel 或 /web/cmd_vel
  -> command_guard_node
  -> /cmd_vel
  -> 底盘运动

网页点击保存PCD
  -> POST /api/mapping/save {"map_name":"factory_01"}
  -> web_api_node 调用 /map_save_with_name
  -> FAST-LIO 保存 PCD
  -> 返回保存路径
```

## 后端接口设计

### GET /api/mapping/status

返回建图状态：

```json
{
  "ok": true,
  "active": true,
  "map_name": "factory_01",
  "mapping_cloud_topic": "/Laser_map",
  "mapping_cloud_publishers": 1,
  "save_service": "/map_save_with_name",
  "save_service_available": true,
  "mapping_cloud_points": 12000,
  "mapping_cloud_source_points": 183421,
  "last_save_success": true,
  "last_save_path": "/home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/maps/factory_01.pcd",
  "last_save_message": "Map saved."
}
```

### POST /api/mapping/start

请求：

```json
{
  "map_name": "factory_01"
}
```

行为：

- 设置建图会话为 active。
- 记录地图名称。
- 不启动新 FAST-LIO 进程。

### POST /api/mapping/stop

行为：

- 设置建图会话为 inactive。
- 不停止 FAST-LIO 进程。
- 不自动保存。

### POST /api/mapping/save

请求：

```json
{
  "map_name": "factory_01"
}
```

行为：

- 检查 `/map_save_with_name` 是否可用。
- 调用 FAST-LIO 保存服务。
- 返回保存结果和 PCD 路径。

## 前端页面设计

在控制面板增加“建图”栏目：

```text
地图名称输入框
开始建图
停止建图
保存PCD
高度过滤开关
最高 Z(m)
2D/3D 视图切换
建图状态
```

在地图工具栏增加：

```text
建图点云
```

建图点云使用独立图层，来源是 `/Laser_map`。这是 FAST-LIO 在 RViz 中用于显示累计地图的点云；`/cloud_registered` 只是当前帧配准点云，不作为网页建图模式的主显示源。

网页地图是 2D 俯视投影，使用点云的 `x-y` 坐标绘制。高度过滤只影响网页显示和视野范围计算，不影响 FAST-LIO 建图和 PCD 保存：

```text
高度过滤开启：只显示 z <= 最高 Z(m) 的点，默认 1.5m。
高度过滤关闭：显示 /Laser_map 下采样后的完整点云，适合室外开阔场景。
```

建图模式支持 3D 点云显示：

```text
2D：原有俯视投影，适合手机快速查看建图轮廓。
3D：Three.js/WebGL 渲染 /Laser_map 的 x/y/z 点云，支持旋转、缩放、平移。
```

3D 显示继续使用网页下采样点云和高度过滤；PCD 保存仍由 FAST-LIO 服务完成，保存完整累计地图。

如果浏览器不支持 WebGL，网页会自动禁用 3D 按钮，2D 建图显示和保存功能不受影响。

## 配置项

新增到 `web_control.yaml`：

```yaml
mapping_cloud_topic: /Laser_map
map_save_service: /map_save_with_name
max_mapping_cloud_points: 30000
```

FAST-LIO 需要保证 PCD 保存目录为普通用户可写目录。当前第一版使用：

```yaml
map_file_path: "/home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/maps/fast_lio_map.pcd"
map_save_dir: "/home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/maps"
pcd_save:
  pcd_save_en: true
```

`/map_save_with_name` 使用 `map_save_dir` 加上网页输入的地图名保存，例如：

```text
/home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/maps/factory_01.pcd
```

建议点数限制：

```text
手机端: 12000 - 20000
电脑端: 30000 - 60000
```

第一版默认取 30000 点以内，避免手机端卡顿。

## 使用流程

1. 启动总服务：

```bash
cd /home/nvidia/project/luxi-atlas
source install/setup.bash
ros2 launch luxi_web_control web_navigation.launch.py
```

2. 手机打开：

```text
http://10.42.0.149:8080
```

3. 输入地图名称。

4. 点击“开始建图”。

5. 用方向键控制机器人慢速移动。

6. 确认建图点云增长。

7. 点击“保存PCD”。

8. 页面显示保存路径。

## 风险和注意事项

- 建图时建议速度控制在 `0.1 - 0.4 m/s`，高速会明显影响建图质量。
- `/map_save_with_name` 依赖 FAST-LIO 节点运行且 `pcd_save.pcd_save_en=true`。
- 如果保存失败且提示 `Map save disabled`，需要检查 FAST-LIO 配置。
- 如果保存失败且提示 `/root/ws/... errno=13`，说明运行的 FAST-LIO 仍是旧二进制或旧配置，需要重新编译并确认 `map_save_dir` 已生效。
- 如果保存失败且提示地图为空，需要先移动机器人让 FAST-LIO 的 `/Laser_map` 累积点云。
- 第一版停止建图不会停止 FAST-LIO，只退出网页建图会话。

## 本地验证记录

2026-06-25 已在本机终端验证通过：

```text
GET /api/mapping/status
  mapping_cloud_topic=/Laser_map
  mapping_cloud_publishers=1
  mapping_cloud_available=true
  save_service_available=true

GET /api/map/snapshot
  mapping_cloud_available=true
  mapping_cloud_points>0

POST /api/mapping/save {"map_name":"web_mapping_test_20260625_after_fix"}
  ok=true
  path=/home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/maps/web_mapping_test_20260625_after_fix.pcd

ls -lh /home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/maps/web_mapping_test_20260625_after_fix.pcd
  file exists, size 1.2M
```

2026-06-25 二次修正后验证：

```text
ros2 topic echo /Laser_map --once --no-arr
  frame_id=camera_init
  width>0

GET /api/mapping/status
  mapping_cloud_topic=/Laser_map
  mapping_cloud_publishers=1
  mapping_cloud_available=true

GET /api/map/snapshot
  mapping_cloud.source=/Laser_map
  mapping_cloud.source_points>0

网页建图模式
  显示 FAST-LIO累计地图
  状态栏显示旧地图已隐藏

POST /api/mapping/save {"map_name":"fast_lio_rviz_map_test_20260625"}
  ok=true
  path=/home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/maps/fast_lio_rviz_map_test_20260625.pcd

PCD 文件检查
  file exists
  POINTS 41131
```

2026-06-25 高度过滤验证：

```text
GET /api/map/snapshot
  mapping_cloud.source=/Laser_map
  sample_points=30000
  z<=1.5m visible points=13373
  z>1.5m hidden points=16627

Chromium 页面验证
  建图模式：FAST-LIO累计地图 13373/30000/1381252
  高度过滤 Z<=1.50m
  旧地图已隐藏

POST /api/mapping/save {"map_name":"height_filter_display_test_20260625"}
  ok=true
  path=/home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/maps/height_filter_display_test_20260625.pcd

PCD 文件检查
  file exists
  POINTS 1485971
```

2026-06-25 3D 点云显示验证：

```text
静态资源
  /vendor/three.module.js HTTP 200
  /vendor/jsm/controls/OrbitControls.js HTTP 200

Chromium DOM 验证
  ?view=3d
  view-3d-btn active
  cloud3d-canvas visible
  cloud3d-canvas data-engine="three.js r164"
  map-status 显示 FAST-LIO累计地图

无 WebGL 降级验证
  data-webgl-error="Error creating WebGL context."
  3D button disabled
  2D 建图状态仍正常显示
```

## 第二版扩展

第二版再做进程级建图管理：

```text
POST /api/mapping/start
  -> 启动独立 FAST-LIO mapping launch

POST /api/mapping/stop
  -> 停止 mapping launch

保存成功后
  -> 登记地图列表
  -> 可选择切换到定位地图
```

第二版还应考虑：

- 导航模式和建图模式互斥。
- 保存后自动生成 PGM/YAML 或 OctoMap。
- 网页地图列表管理。
- 异常退出后的残留进程清理。
