# Luxi Atlas 启动说明

## 重启设备后启动网页控制系统

设备重启后，在终端执行下面这一条命令即可启动完整网页控制系统：

```bash
cd /home/nvidia/project/luxi-atlas && source third_party/go2_ros2/setup.sh && source install/setup.bash && WIFI_IP=$(ip -4 -o addr show wlP1p1s0 | awk '{split($4,a,"/"); print a[1]; exit}') && echo "Android Web URL: http://${WIFI_IP}:8082" && ros2 launch luxi_web_control web_navigation.launch.py wifi_interface:=wlP1p1s0 bind_address:=0.0.0.0 start_legacy_http:=false
```

启动后手机和设备连接到同一个 WiFi，在手机浏览器打开终端打印的地址，例如当前设备一般是：

```text
http://10.42.0.149:8082
```

该启动命令会拉起：

- 雷达驱动和 FAST-LIO 建图节点。
- RGB 相机节点。
- 定位节点。
- 导航规划节点。
- C++ 网页/API 服务，端口 `8082`，安卓手机推荐直接访问这个端口。
- 推荐启动命令会显式传入 `start_legacy_http:=false`，不启动旧版 Python 静态网页服务端口 `8080`；只有改为 `start_legacy_http:=true` 才会启动。
- 网页控制速度输出，默认转发到 `/cmd_vel`。

## 停止网页控制系统

切换 WiFi、重新启动服务、重新插拔雷达或发现启动异常前，必须先停止旧进程。否则可能出现：

- `8082` 端口已被旧 `web_api_node` 占用。
- Livox 雷达驱动重复启动，导致雷达打开失败。
- FAST-LIO、定位、地图发布节点重复运行，网页看到的数据混乱。

如果启动命令还在当前终端前台运行，优先按：

```bash
Ctrl+C
```

等待 5 到 10 秒后检查是否已停止：

```bash
pgrep -af "web_navigation.launch.py|web_api_node|livox_ros_driver2_node|fastlio_mapping|luxi_grid_map_publisher|luxi_open3d_localization|luxi_navigation_node"
ss -ltnp | grep ':8082' || echo "8082 已释放"
```

如果终端已经关闭，或者 `Ctrl+C` 后仍有残留进程，执行下面这组命令清理旧服务：

```bash
pkill -INT -f "web_navigation.launch.py|web_api_node|livox_ros_driver2_node|fastlio_mapping|luxi_grid_map_publisher|luxi_open3d_localization|luxi_navigation_node"
sleep 8
pkill -TERM -f "web_navigation.launch.py|web_api_node|livox_ros_driver2_node|fastlio_mapping|luxi_grid_map_publisher|luxi_open3d_localization|luxi_navigation_node"
sleep 3
pgrep -af "web_navigation.launch.py|web_api_node|livox_ros_driver2_node|fastlio_mapping|luxi_grid_map_publisher|luxi_open3d_localization|luxi_navigation_node" || echo "网页控制系统相关进程已停止"
ss -ltnp | grep ':8082' || echo "8082 已释放"
```

确认没有旧进程、`8082` 已释放后，再重新执行启动命令。

## 网段或 IP 变化

如果设备连接到新的 WiFi，局域网网段可能从 `10.42.0.x` 变成其他网段，例如 `192.168.x.x`。

不需要修改代码。继续使用上面的启动命令即可，因为命令会自动读取 `wlP1p1s0` 当前 WiFi IP 并打印安卓访问地址。网页/API 服务监听 `0.0.0.0:8082`，WiFi 地址变化后重启服务即可恢复访问。

切换 WiFi 推荐流程：

```bash
# 1. 先停止旧服务，避免端口和雷达驱动冲突
pkill -INT -f "web_navigation.launch.py|web_api_node|livox_ros_driver2_node|fastlio_mapping|luxi_grid_map_publisher|luxi_open3d_localization|luxi_navigation_node"
sleep 8
pkill -TERM -f "web_navigation.launch.py|web_api_node|livox_ros_driver2_node|fastlio_mapping|luxi_grid_map_publisher|luxi_open3d_localization|luxi_navigation_node"
sleep 3

# 2. 确认 WiFi 已获得新 IP
ip -4 -o addr show wlP1p1s0

# 3. 重新启动网页控制系统
cd /home/nvidia/project/luxi-atlas && source third_party/go2_ros2/setup.sh && source install/setup.bash && WIFI_IP=$(ip -4 -o addr show wlP1p1s0 | awk '{split($4,a,"/"); print a[1]; exit}') && echo "Android Web URL: http://${WIFI_IP}:8082" && ros2 launch luxi_web_control web_navigation.launch.py wifi_interface:=wlP1p1s0 bind_address:=0.0.0.0 start_legacy_http:=false
```

也可以手动查看当前 WiFi IP：

```bash
ip -4 -o addr show wlP1p1s0 | awk '{split($4,a,"/"); print a[1]; exit}'
```

如果 WiFi 网卡名字发生变化，先查看网卡：

```bash
ip -4 addr
```





然后把启动命令中的 `wlP1p1s0` 替换成新的 WiFi 网卡名。

## 网络接口约定

- `wlP1p1s0`：WiFi，用于手机访问网页和局域网通信。
- `enP8p1s0`：有线以太网，保留给雷达使用，不作为网页控制网络。

不要把网页服务绑定到 `enP8p1s0`，否则手机端可能无法访问，且可能影响雷达通信。

## 常用检查命令

查看网页服务是否启动：

```bash
ss -ltnp | grep ':8082'
```

正常应看到类似：

```text
0.0.0.0:8082
```

检查安卓网页入口：

```bash
curl -I http://$(ip -4 -o addr show wlP1p1s0 | awk '{split($4,a,"/"); print a[1]; exit}'):8082/
```

检查网页 API：

```bash
curl -sS http://$(ip -4 -o addr show wlP1p1s0 | awk '{split($4,a,"/"); print a[1]; exit}'):8082/api/status
```

查看已保存地图列表：

```bash
curl -sS http://$(ip -4 -o addr show wlP1p1s0 | awk '{split($4,a,"/"); print a[1]; exit}'):8082/api/maps
```

## 已保存地图位置

网页点击“保存PCD”后，地图保存到：

```text
/home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/maps
```

例如：

```text
/home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/maps/test_map.pcd
```

网页选择已保存地图后，如果该地图还没有转换成栅格地图，系统会自动转换并生成：

```text
/home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/tools/transfrom_tools/pcd_to_pgm/output/<地图名>_nav_floor_plane_autofit_clean20/map.pgm
/home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/tools/transfrom_tools/pcd_to_pgm/output/<地图名>_nav_floor_plane_autofit_clean20/map.yaml
/home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/tools/transfrom_tools/pcd_to_pgm/output/<地图名>_nav_floor_plane_autofit_clean20/map.json
```

## 直接打开指定地图

如果要直接打开某张离线地图并进入 3D 点云查看，可以使用：

```text
http://<设备WiFi_IP>:8082/?map=test_map&view=3d
```

当前示例：

```text
http://10.42.0.149:8082/?map=test_map&view=3d
```
