# 02-adj_visualization

这是第二个 **纯 C++** sample。

这个 sample 的目标首先是“稳定看到实时画面”，所以程序启动后会先主动把相机切到：

- `AcquisitionMode=Continuous`
- `TriggerMode=Off`

这样能保证预览窗口默认是连续出图状态。进入界面后，你仍然可以再把触发模式、触发源、边沿等参数改成你需要的值。

它会把界面拆成两部分：

- 左侧：实时相机预览，预览区域固定为 `1024x640`
- 右侧：参数调节面板，可以直接调相机参数

当前面板覆盖这些核心参数：

- 分辨率：`Width`、`Height`
- ROI：`OffsetX`、`OffsetY`
- 像素格式：`PixelFormat`
- 采集模式：`AcquisitionMode`
- 触发模式：`TriggerMode`
- 触发源：`TriggerSource`
- 触发边沿：`TriggerActivation`
- 自动曝光：`ExposureAuto`
- 自动曝光范围：`AutoExposureTimeLowerLimit`、`AutoExposureTimeUpperLimit`
- 自动曝光目标：`AutoTargetBrightness`
- 曝光时间：`ExposureTime`
  当前 UI 中手动曝光时间滑条限制为 `1000 us` 到 `5 s`
- 自动增益：`GainAuto`
- 增益：`Gain`

界面交互方式：

- 枚举参数：点击右侧按钮切换
- 数值参数：拖动右侧滑条，松开鼠标后应用
- 鼠标滚轮：滚动右侧参数面板
- 右侧竖向滚动条：支持直接拖动，也可以点击轨道跳转
- `s`：保存当前预览帧
- `r`：重新从相机读取当前参数
- `q`：退出

## 自动曝光建议

这台相机在 `GainAuto=Continuous` 时，画面通常会显得更亮、层次更差，现场观感往往偏“过曝不好看”。一般不建议长期依赖自动增益来控亮度。

更推荐的思路是：

- 开启自动曝光：`ExposureAuto=Continuous`
- 关闭自动增益：`GainAuto=Off`
- 把 `Gain` 固定在较低值，例如 `0`

如果把 `ExposureAuto` 打开后画面仍然明显过曝，优先这样调：

- 保持 `ExposureAuto=Continuous`
- 下调 `AutoTargetBrightness`
- 下调 `AutoExposureTimeUpperLimit`，限制自动曝光允许拉到的最长曝光时间

对你这台相机来说，更稳妥的起点通常是：

- `GainAuto=Off`
- `Gain=0`
- `ExposureAuto=Continuous`
- `AutoExposureTimeUpperLimit` 先明显小于当前手动曝光值 `95070 us`
- 如果切回手动曝光，`ExposureTime` 建议先在 `1000 us` 到 `5000000 us` 这个 UI 限定区间内调节

## 当前相机读回示例

当前这台相机在开发现场读回来的关键配置是：

- 分辨率：`4096 x 3000`
- ROI：`OffsetX=0`，`OffsetY=0`
- 像素格式：`BayerGB8`
- 采集模式：`Continuous`
- 触发模式：`Off`
- 触发源：`Line0`
- 触发边沿：`RisingEdge`
- 自动曝光：`Off`
- 曝光时间：`95070 us`
- 自动增益：`Off`
- 增益：`0`

需要注意：

- 程序启动时会先强制切到连续预览模式，所以如果你之前把相机留在触发模式，启动后会被切回 `TriggerMode=Off`
- 启动完成后，右侧面板显示的是“切到连续预览之后的当前相机状态”
- 你点击按钮或拖动滑条后，参数会立即写回相机
- 如果你把 `TriggerMode` 改成 `On`，但没有给相机外部触发信号，左侧画面停止刷新是预期现象，不是程序卡死

## 构建

```bash
cd /root/ws/MV_HIKROBOT/MvCamCtrlSDK_Runtime-4.7.0_x86_64_20251113/luxitech/Project/02-adj_visualization
mkdir -p build
cd build
cmake ..
make -j
```

## 运行

```bash
cd /root/ws/MV_HIKROBOT/MvCamCtrlSDK_Runtime-4.7.0_x86_64_20251113/luxitech/Project/02-adj_visualization/build
./02-adj_visualization
```

## 只做抓图验证

如果当前环境没有图形桌面，可以先用：

```bash
./02-adj_visualization --headless --save /tmp/adj_preview.png
```
