# MV-CU013-A0UC 02-adj_visualization

这是面向 **Hikrobot MV-CU013-A0UC** 的实时预览示例。

当前版本直接显示 `1280 x 1024` 原帧，不做缩放。当前 ARM 系统上的 OpenCV `imgproc` 在 `resize`/`putText` 调用中会段错误，所以预览窗口不绘制文字面板；动态调参通过 OpenCV 窗口按键和终端命令完成。

保留的运行功能：

- OpenCV 窗口按键动态调参
- 终端输入 `list`：列出当前可调参数和值
- 终端输入 `set <Key> <Value>`：动态写入参数
- 终端输入 `save [path]`：保存当前帧
- 终端输入 `quit`：退出程序
- `--headless --save PATH`：无窗口抓图验证

常用终端命令示例：

```text
list
set ExposureAuto Off
set ExposureTime 8000
set GainAuto Off
set GainAuto On
set Gain 0
set TriggerMode Off
set PixelFormat BayerGB8
save /tmp/cu013_adj_frame.png
quit
```

`ExposureAuto` 和 `GainAuto` 的 SDK 原始取值是 `Off/Once/Continuous`；为了方便测试，终端里的 `On`、`ON`、`true`、`1` 会按 `Continuous` 处理。

OpenCV 窗口按键：

```text
[ / ]  ExposureTime -/+ 1000 us
- / =  Gain -/+ 0.5
a      toggle ExposureAuto Off/Continuous
g      toggle GainAuto Off/Continuous
p      cycle PixelFormat
l      list current settings in terminal
h      show key help in terminal
s      save current frame
r      reload settings
q      quit
```

## 构建

```bash
cd /home/nvidia/project/luxi-atlas/device/luxi-hikrobot/Project/MV-CU013-A0UC/02-adj_visualization
cmake -S . -B build-arm -DMVS_SDK_PATH=/home/nvidia/project/luxi-atlas/device/luxi-hikrobot/SDK/runtime/MvCamCtrlSDK
cmake --build build-arm -j
```

## 运行

```bash
source /home/nvidia/project/luxi-atlas/device/luxi-hikrobot/SDK/extracted/MvCamCtrlSDK_Runtime-4.8.0_aarch64_20260512/set_env_path.sh \
  /home/nvidia/project/luxi-atlas/device/luxi-hikrobot/SDK/runtime/MvCamCtrlSDK 4.8.0

./build-arm/02-adj_visualization
```

无窗口抓图验证：

```bash
./build-arm/02-adj_visualization --headless --save /tmp/cu013_adj_frame.png
```
