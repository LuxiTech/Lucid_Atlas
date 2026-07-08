# MV-CU013-A0UC 01-mini_visualization

这是面向 **Hikrobot MV-CU013-A0UC** 的最小预览/抓图示例。

当前验证设备：

- 型号：`MV-CU013-A0UC`
- 序列号：`DB1355165`
- Runtime：`MvCamCtrlSDK 4.8.0`
- 分辨率：`1280 x 1024`
- 默认曝光：`ExposureAuto=Off`，`ExposureTime=25000 us`
- 默认增益：`GainAuto=Off`，`Gain=15`

CU013 预览不做缩放，窗口直接按 `1280 x 1024` 显示原帧。系统当前 OpenCV `imgproc` 的 `resize`/`putText` 路径在 ARM 环境会段错误，所以本示例避免调用这些显示缩放接口。

## 构建

```bash
cd /home/nvidia/project/luxi-atlas/device/luxi-hikrobot/Project/MV-CU013-A0UC/01-mini_visualization
cmake -S . -B build-arm -DMVS_SDK_PATH=/home/nvidia/project/luxi-atlas/device/luxi-hikrobot/SDK/runtime/MvCamCtrlSDK
cmake --build build-arm -j
```

## 运行

```bash
source /home/nvidia/project/luxi-atlas/device/luxi-hikrobot/SDK/extracted/MvCamCtrlSDK_Runtime-4.8.0_aarch64_20260512/set_env_path.sh \
  /home/nvidia/project/luxi-atlas/device/luxi-hikrobot/SDK/runtime/MvCamCtrlSDK 4.8.0

./build-arm/01-mini_visualization
```

无窗口抓图验证：

```bash
./build-arm/01-mini_visualization --headless --save /tmp/cu013_first_frame.png
```

覆盖默认曝光/增益：

```bash
./build-arm/01-mini_visualization \
  --exposure-auto Off \
  --exposure-us 25000 \
  --gain-auto Off \
  --gain 15
```
