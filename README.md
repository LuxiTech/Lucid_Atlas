# Luxi Atlas 总体工程说明

Luxi Atlas 是面向 Unitree Go2 机器狗、Livox MID360 雷达、海康 RGB 相机、FAST-LIO/LIVO 建图、Open3D 定位、二维导航和网页控制的一体化 ROS 2 工程。当前目录已经整理为一个统一 Git 仓库，原来各子项目中的 `.git` 元数据已移除，源码目录仍保持原路径，便于继续按现有启动脚本和包路径使用。

## 仓库组织原则

- 根目录是唯一 Git 仓库入口，后续提交统一在本目录完成。
- `project/`、`device/`、`third_party/` 是主要跟踪内容。
- `demo/` 保留在本地作为历史示例和参考工程，但通过根 `.gitignore` 整体不跟踪。
- 各层 `build/`、`install/`、`log/`、`output/`、`Log/` 等编译、运行和标定输出目录不跟踪。
- 原子仓库的 README 和 `.gitignore` 文件会保留，作为各模块的局部说明和历史约定参考。

## 顶层目录结构

| 路径 | 是否跟踪 | 说明 |
| --- | --- | --- |
| `project/` | 是 | Luxi Atlas 自研 ROS 2 功能包，包含建图、定位、导航和网页控制。 |
| `device/` | 是 | 设备驱动与传感器适配，包括 MID360 雷达和海康相机。 |
| `third_party/` | 是 | 第三方或二次开发依赖，包括 Go2 ROS 2、标定、vikit、Open3D。 |
| `deploy/` | 是 | 部署相关配置，例如 systemd 服务文件。 |
| `scripts/` | 是 | 仓库级辅助脚本。 |
| `demo/` | 否 | 历史 demo 和参考项目，当前不纳入大仓库版本管理。 |
| `build/`、`install/`、`log/` | 否 | 根工作区的 colcon 编译、安装和日志目录。 |
| `.local_ros_debs/` | 否 | 本地 ROS deb 包缓存和临时根目录。 |
| `MvSdkLog/`、`nohup.out` | 否 | 运行期日志。 |

## 核心工程模块

### `project/luxi-atlas-livo`

FAST-LIVO 相关建图工程，负责融合 MID360 点云和 RGB 图像数据，支持连续建图、RGB 点云显示、时间差自动校准等功能。该模块依赖雷达驱动、相机节点、vikit 相关库和系统标定参数。

### `project/luxi-atlas-lio`

FAST-LIO 建图与地图处理工程，当前包名为 `fast_lio`。仓库中包含 PCD 到栅格地图转换、Octomap/OccupancyGrid 转换和地图保存相关工具，是导航和定位地图来源之一。

### `project/luxi-localization`

Open3D 全局定位模块，包名为 `luxi_localization`。用于加载 FAST-LIO 生成的地图，通过 ICP 等方式输出机器人初始或持续定位结果，并提供 2D 栅格定位显示和地图参数化配置。

### `project/luxi-navigation`

二维导航规划模块，包名为 `luxi_navigation`。负责基于地图、定位和目标点生成导航路径，并输出局部规划控制相关信息。当前包含 DWB 局部规划输出和第一阶段全局导航规划逻辑。

### `project/luxi-web-control`

网页控制和 API 服务模块，包名为 `luxi_web_control`。提供手机网页端地图显示、摇杆控制、导航目标下发、状态展示和 C++ Web/API 服务。默认 API 端口为 `8082`。

## 设备模块

### `device/mid360`

Livox MID360 雷达相关工程，包含 Livox ROS Driver 2、FAST-LIO ROS 2 适配、外部同步配置和地图保存配置。该模块通常需要配合 `192.168.123.0/24` 有线网络使用。

### `device/luxi-hikrobot`

海康 MV-CU013-A0UC 相机适配工程，包含 ROS 2 相机节点、动态参数配置、压缩图像发布配置和相机 SDK 相关接入。实际 SDK、日志和本地编译产物不纳入 Git。

## 第三方与依赖模块

### `third_party/go2_ros2`

Unitree Go2 ROS 2 集成工程，包含 CycloneDDS 配置、Unitree 消息包、Go2 控制、建图、导航示例，以及本项目使用的 `go2_cmd_vel_bridge`。其中 `setup.sh` 用于准备 Go2 相关环境变量。

### `third_party/lux_-atlas_-calib`

Luxi Atlas 标定工程，包名为 `fast_calib`。用于 MID360 与 CU013 相机联合标定，输出标定参数和相关中间结果。`output/`、`Log/`、`calib_data/` 等运行产物不纳入 Git。

### `third_party/rpg_vikit`

vikit 基础库集合，包括 `vikit_common`、`vikit_py`、`vikit_ros`，为视觉惯性/视觉雷达相关算法提供公共工具和 ROS 封装。

### `third_party/open3d`

Open3D 第三方源码，用于点云处理、ICP 配准、地图处理和定位相关能力。该目录通常不直接在根工作区频繁修改，除非定位或构建链路需要同步上游修复。

## Demo 目录

`demo/` 下保留以下参考项目：

- `demo/lux-atlas`
- `demo/LIV_handhold`
- `demo/jie_3d_nav`
- `demo/luckrobot`

这些目录目前用于查阅历史实现、启动方式和实验代码，不作为当前 Luxi Atlas 大仓库的正式跟踪内容。根 `.gitignore` 已配置 `demo/`，因此新仓库不会提交 demo 代码。如果后续某个 demo 需要转正，应先从 `demo/` 迁移到 `project/`、`device/` 或 `third_party/` 的合适位置，再按正式模块维护。

## 编译与环境

推荐在仓库根目录执行构建：

```bash
cd /home/nvidia/project/luxi-atlas
source /opt/ros/humble/setup.bash
source third_party/go2_ros2/setup.sh
colcon build --symlink-install
source install/setup.bash
```

如果只编译核心自研包，可使用：

```bash
colcon build --symlink-install \
  --packages-select fast_livo fast_lio luxi_localization luxi_navigation luxi_web_control
```

Go2 相关包、MID360 驱动、海康相机节点和 Open3D 依赖可能需要额外系统库或 SDK。遇到构建问题时，优先查看对应子目录 README：

- `third_party/go2_ros2/README.md`
- `device/mid360/README.md`
- `device/luxi-hikrobot/README.md`
- `project/luxi-atlas-livo/README.md`
- `project/luxi-atlas-lio/README.md`
- `project/luxi-localization/README.md`

## 网页控制系统启动

设备重启后，在终端执行下面命令启动完整网页控制系统：

```bash
cd /home/nvidia/project/luxi-atlas && source third_party/go2_ros2/setup.sh && source install/setup.bash && WIFI_IP=$(ip -4 -o addr show wlP1p1s0 | awk '{split($4,a,"/"); print a[1]; exit}') && echo "Android Web URL: http://${WIFI_IP}:8082" && ros2 launch luxi_web_control web_navigation.launch.py wifi_interface:=wlP1p1s0 bind_address:=0.0.0.0 start_legacy_http:=false
```

启动后，手机和设备连接到同一个 WiFi，在手机浏览器打开终端打印的地址，例如：

```text
http://10.42.0.149:8082
```

该 launch 会拉起：

- Livox 雷达驱动和 FAST-LIO 建图节点。
- RGB 相机节点。
- Open3D 定位节点。
- 导航规划节点。
- C++ 网页/API 服务，默认端口 `8082`。
- 网页控制速度输出，默认转发到 `/cmd_vel`。

推荐启动命令会显式传入 `start_legacy_http:=false`，不启动旧版 Python 静态网页服务端口 `8080`；只有改为 `start_legacy_http:=true` 才会启动。

## Go2 机器狗控制链路

Go2 控制链路使用用户级 systemd 服务：

```text
luxi-go2-cmd-vel-bridge.service
```

控制链路如下：

```text
网页摇杆/导航 -> /web/cmd_vel -> /cmd_vel -> go2_cmd_vel_bridge -> /api/sport/request -> Go2
```

常用操作：

```bash
systemctl --user status luxi-go2-cmd-vel-bridge.service
systemctl --user start luxi-go2-cmd-vel-bridge.service
systemctl --user stop luxi-go2-cmd-vel-bridge.service
systemctl --user restart luxi-go2-cmd-vel-bridge.service
systemctl --user enable luxi-go2-cmd-vel-bridge.service
systemctl --user disable luxi-go2-cmd-vel-bridge.service
```

确认 Go2 DDS topic 已发现：

```bash
cd /home/nvidia/project/luxi-atlas
source third_party/go2_ros2/setup.sh
source third_party/go2_ros2/luxitech/go2/go2_robot2/install/setup.bash
ros2 topic list | grep -E "sportmodestate|api/sport|utlidar"
```

正常应能看到：

```text
/lf/sportmodestate
/sportmodestate
/api/sport/request
/api/sport/response
```

## 停止网页控制系统

切换 WiFi、重新启动服务、重新插拔雷达或发现启动异常前，先停止旧进程，避免端口占用、雷达重复打开或定位/建图节点重复运行。

如果启动命令还在当前终端前台运行，优先按 `Ctrl+C`。如果终端已经关闭，或 `Ctrl+C` 后仍有残留进程，执行：

```bash
pkill -INT -f "web_navigation.launch.py|web_api_node|livox_ros_driver2_node|fastlio_mapping|luxi_grid_map_publisher|luxi_open3d_localization|luxi_navigation_node"
sleep 8
pkill -TERM -f "web_navigation.launch.py|web_api_node|livox_ros_driver2_node|fastlio_mapping|luxi_grid_map_publisher|luxi_open3d_localization|luxi_navigation_node"
sleep 3
pgrep -af "web_navigation.launch.py|web_api_node|livox_ros_driver2_node|fastlio_mapping|luxi_grid_map_publisher|luxi_open3d_localization|luxi_navigation_node" || echo "网页控制系统相关进程已停止"
ss -ltnp | grep ':8082' || echo "8082 已释放"
```

## Git 管理约定

提交信息沿用当前各子项目已有风格，推荐使用以下前缀：

- `[func]` 新功能、功能增强。
- `[fix]` Bug 修复、参数修正、兼容性修复。
- `[docs]` README、部署说明、注释文档更新。
- `[refactor]` 结构调整、代码重构，不改变外部行为。
- `[chore]` 构建、忽略规则、仓库维护类变更。

示例：

```text
[func]新增网页导航状态显示
[fix]修正MID360外部同步配置
[docs]更新统一仓库结构说明
[chore]整理monorepo忽略规则
```

## 不纳入 Git 的内容

根 `.gitignore` 已统一忽略：

- `demo/`
- `.agents/`、`.codex/`
- `.local_ros_debs/`
- `build/`、`install/`、`log/`
- 任意子目录下的 `build/`、`install/`、`log/`
- `output/`、`outputs/`、`Log/`
- `MvSdkLog/`、`nohup.out`、`*.log`
- Python 缓存、虚拟环境、CMake 临时文件、编辑器配置
- 各子项目原有 `.gitignore` 中额外排除的本地数据，例如标定 `calib_data/`、PCD 地图输出、Open3D 文档生成物等

如果某个地图、点云或 demo 资源需要随版本发布，应放入专门的数据发布流程或显式调整忽略规则后再提交。
