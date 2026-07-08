# 01-mini_visualization

这是第一个 **纯 C++** sample。

目标很简单：

- 打开第一台 Hikrobot 相机
- 设置连续出图
- 抓一帧
- 保存到文件
- 如果不是 `--headless`，再开一个 OpenCV 预览窗口

预览窗口固定为 `1024x640`，原始画面按比例缩放后居中显示，不会拉伸变形。

## 当前相机设置说明

这里把两类信息分开写清楚：

### 1. sample 默认会主动设置的内容

当前 `01-mini_visualization` 在不带额外参数运行时，会主动做这两件事：

- 尝试设置 `AcquisitionMode=Continuous`
- 强制设置 `TriggerMode=Off`

这意味着 sample 的目标是让相机进入“连续自由出图”状态。

它 **不会默认修改** 下面这些参数，除非你显式传了命令行参数：

- `Width`
- `Height`
- `ExposureAuto`
- `ExposureTime`
- `GainAuto`
- `Gain`

### 2. 这台相机当前读回的实际参数

以下是当前从这台相机直接读回来的关键配置：

- 相机型号：`MV-CH120-60UC`
- 序列号：`DA7672183`
- 分辨率：`4096 x 3000`
- ROI 偏移：`OffsetX=0`，`OffsetY=0`
- 像素格式：`BayerGB8`
- 采集模式：`Continuous`
- 触发模式：`Off`
- 触发源：`Line0`
- 触发边沿：`RisingEdge`
- 自动曝光：`Off`
- 曝光时间：`95070 us`
- 自动增益：`Off`
- 增益：`0`

这里有个细节值得说明：

- `TriggerMode=Off` 时，相机会连续出图
- 即使 `TriggerSource=Line0`、`TriggerActivation=RisingEdge` 仍然保留在设备节点里，只要 `TriggerMode` 关闭，这两个值当前就不会参与实际出图流程

也就是说，按现在这组配置，sample 跑起来后相机会直接连续出图，当前不是触发采图模式。

## 构建

```bash
cd /root/ws/MV_HIKROBOT/MvCamCtrlSDK_Runtime-4.7.0_x86_64_20251113/luxitech/Project/01-mini_visualization
mkdir -p build
cd build
cmake ..
make -j
```

## 运行

```bash
cd /root/ws/MV_HIKROBOT/MvCamCtrlSDK_Runtime-4.7.0_x86_64_20251113/luxitech/Project/01-mini_visualization/build
./01-mini_visualization --headless
```

默认会把第一帧保存到：

```bash
output/first_frame.png
```

## 常用参数

```bash
./01-mini_visualization \
  --index 0 \
  --width 1920 \
  --height 1080 \
  --exposure-auto Off \
  --exposure-us 10000 \
  --gain-auto Off \
  --gain 6 \
  --save /tmp/first_frame.png \
  --headless
```

如果要按序列号打开：

```bash
./01-mini_visualization --serial DA7672183 --headless
```
