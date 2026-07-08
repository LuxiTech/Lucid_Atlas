# Luxitech MVS SDK Notes

## 1. 当前开发方式

当前 `luxitech` 目录已经切到 **纯 C++ 开发**，不通过 Python 调用相机。

运行时依赖仍然是官方 MVS Runtime：

- SDK 根目录：`/opt/MVS`
- 主动态库：`/opt/MVS/lib/64/libMvCameraControl.so`

由于当前工作区的 runtime 包没有附带官方 `MvCameraControl.h` / `CameraParams.h`，所以本项目在本地维护了一份 **最小可用 C++ 头文件子集**，只覆盖当前 BSP 和 sample 要用到的结构体、常量和函数声明。

相关路径：

- 类型定义：[mv_sdk_types.hpp](/root/ws/MV_HIKROBOT/MvCamCtrlSDK_Runtime-4.7.0_x86_64_20251113/luxitech/bsp/include/luxitech/mvs/mv_sdk_types.hpp)
- C 接口声明：[mv_sdk_api.hpp](/root/ws/MV_HIKROBOT/MvCamCtrlSDK_Runtime-4.7.0_x86_64_20251113/luxitech/bsp/include/luxitech/mvs/mv_sdk_api.hpp)
- C++ BSP 封装：[mvs_camera.hpp](/root/ws/MV_HIKROBOT/MvCamCtrlSDK_Runtime-4.7.0_x86_64_20251113/luxitech/bsp/include/luxitech/mvs/mvs_camera.hpp) / [mvs_camera.cpp](/root/ws/MV_HIKROBOT/MvCamCtrlSDK_Runtime-4.7.0_x86_64_20251113/luxitech/bsp/src/mvs_camera.cpp)

## 2. 这些函数和结构体是从哪里获取到的

当前这套 C++ 头文件子集来自三类来源交叉核对：

### 2.1 `libMvCameraControl.so` 导出符号

用于确认哪些 `MV_CC_*` 接口在 Linux Runtime 中真实存在。

命令：

```bash
nm -D /opt/MVS/lib/64/libMvCameraControl.so | c++filt | rg ' T MV_CC_'
```

### 2.2 `libMvCameraControl.so` 字符串表

用于确认常见 GenICam 节点名，比如：

- `TriggerMode`
- `TriggerSource`
- `TriggerActivation`
- `ExposureAuto`
- `ExposureTime`
- `GainAuto`
- `Gain`
- `AcquisitionMode`
- `Width`
- `Height`
- `OffsetX`
- `OffsetY`
- `PixelFormat`

命令：

```bash
strings -n 6 /opt/MVS/lib/64/libMvCameraControl.so \
  | rg 'Trigger|Exposure|Gain|Acquisition|PixelFormat|Width|Height|Offset'
```

### 2.3 Python `ctypes` 绑定，仅作为结构体参考

当前仓库保留了一份 vendored `MvImport`：

[vendor/MvImport](/root/ws/MV_HIKROBOT/MvCamCtrlSDK_Runtime-4.7.0_x86_64_20251113/luxitech/bsp/vendor/MvImport)

它 **不是运行时依赖**，只是用来反查这些 Linux ABI 相关信息：

- `MV_CC_DEVICE_INFO`
- `MV_CC_DEVICE_INFO_LIST`
- `MV_FRAME_OUT_INFO_EX`
- `MV_CC_PIXEL_CONVERT_PARAM`
- `MVCC_INTVALUE_EX`
- `MVCC_FLOATVALUE`
- `MVCC_ENUMVALUE`
- `MVCC_ENUMENTRY`

也就是说：

- 开发和 sample 现在走的是 **C++ + `libMvCameraControl.so`**
- `MvImport/*.py` 只是在缺官方头文件时，帮助我们把 Linux 下结构体尺寸和字段顺序核出来

## 3. 当前 C++ 里可以直接用的官方 SDK 接口

这些接口已经在 [mv_sdk_api.hpp](/root/ws/MV_HIKROBOT/MvCamCtrlSDK_Runtime-4.7.0_x86_64_20251113/luxitech/bsp/include/luxitech/mvs/mv_sdk_api.hpp) 里声明好了：

### 3.1 生命周期和设备管理

- `MV_CC_Initialize`
- `MV_CC_Finalize`
- `MV_CC_GetSDKVersion`
- `MV_CC_EnumDevices`
- `MV_CC_CreateHandle`
- `MV_CC_OpenDevice`
- `MV_CC_CloseDevice`
- `MV_CC_DestroyHandle`

### 3.2 取流

- `MV_CC_StartGrabbing`
- `MV_CC_StopGrabbing`
- `MV_CC_GetOneFrameTimeout`

### 3.3 参数设置

- `MV_CC_SetEnumValueByString`
- `MV_CC_SetFloatValue`
- `MV_CC_SetIntValueEx`
- `MV_CC_SetCommandValue`

### 3.4 参数读取

- `MV_CC_GetIntValue`
- `MV_CC_GetIntValueEx`
- `MV_CC_GetFloatValue`
- `MV_CC_GetEnumValue`
- `MV_CC_GetEnumEntrySymbolic`

### 3.5 图像转换

- `MV_CC_ConvertPixelType`

### 3.6 GigE 优化

- `MV_CC_GetOptimalPacketSize`

## 4. 当前 BSP 已封装的通用 C++ 接口

高层入口是：

- [mvs_camera.hpp](/root/ws/MV_HIKROBOT/MvCamCtrlSDK_Runtime-4.7.0_x86_64_20251113/luxitech/bsp/include/luxitech/mvs/mvs_camera.hpp)

核心能力：

- `MvsCamera::sdk_version()`
- `MvsCamera::enumerate_devices()`
- `open(index, serial_number)`
- `close()`
- `configure_continuous_output()`
- `configure_software_trigger()`
- `configure_line_trigger(line_name, activation)`
- `set_auto_exposure(mode, lower_us, upper_us)`
- `set_manual_exposure(exposure_us)`
- `set_gain_auto(mode)`
- `set_manual_gain(gain_value)`
- `set_enum(key, value)`
- `set_int(key, value)`
- `set_float(key, value)`
- `set_command(key)`
- `start_grabbing()`
- `stop_grabbing()`
- `trigger_software()`
- `grab_frame_bgr(timeout_ms)`
- `save_frame(frame, output_path)`

## 5. 常用参数怎么在 C++ 里设置

### 5.1 连续出图

```cpp
camera.configure_continuous_output();
```

内部等价于：

```cpp
camera.set_enum("AcquisitionMode", "Continuous");
camera.set_enum("TriggerMode", "Off");
```

其中 `AcquisitionMode` 如果设备不暴露，不会阻塞整体流程；`TriggerMode=Off` 才是“让相机自由出图”的关键。

### 5.2 软触发

```cpp
camera.configure_software_trigger();
camera.start_grabbing();
camera.trigger_software();
```

等价节点：

- `TriggerMode = On`
- `TriggerSource = Software`
- `TriggerSoftware` 命令节点执行一次

### 5.3 外部线触发和采样边沿

```cpp
camera.configure_line_trigger("Line0", "RisingEdge");
```

常见可选值：

- `Line0`
- `Line1`
- `Line2`
- `Line3`

常见边沿值：

- `RisingEdge`
- `FallingEdge`
- `AnyEdge`
- `LevelHigh`
- `LevelLow`

### 5.4 自动曝光和手动曝光

自动曝光：

```cpp
camera.set_auto_exposure("Continuous", 100.0f, 30000.0f);
```

手动曝光：

```cpp
camera.set_manual_exposure(10000.0f);
```

### 5.5 自动增益和手动增益

自动增益：

```cpp
camera.set_gain_auto("Continuous");
```

手动增益：

```cpp
camera.set_manual_gain(6.0f);
```

### 5.6 分辨率和 ROI

```cpp
camera.set_int("Width", 1920);
camera.set_int("Height", 1080);
camera.set_int("OffsetX", 0);
camera.set_int("OffsetY", 0);
```

## 6. 第一个 sample 在哪里

sample 路径：

- [main.cpp](/root/ws/MV_HIKROBOT/MvCamCtrlSDK_Runtime-4.7.0_x86_64_20251113/luxitech/Project/01-mini_visualization/main.cpp)
- [CMakeLists.txt](/root/ws/MV_HIKROBOT/MvCamCtrlSDK_Runtime-4.7.0_x86_64_20251113/luxitech/Project/01-mini_visualization/CMakeLists.txt)
- [README.md](/root/ws/MV_HIKROBOT/MvCamCtrlSDK_Runtime-4.7.0_x86_64_20251113/luxitech/Project/01-mini_visualization/README.md)

这个 sample 做的事情：

1. 枚举相机
2. 打开第一台相机
3. 设置连续出图
4. 开始取流
5. 抓第一帧
6. 保存图片
7. 非 `--headless` 模式下打开预览窗口

## 7. 当前这套 C++ 方案的边界

- 现在这套本地头文件是 **最小可用子集**
- 它已经够我们做：枚举、打开、调触发、调曝光、调增益、抓图、转 BGR、保存图像
- 如果后面拿到官方 `Development/Includes`，可以把这些本地头文件替换成官方版本，BSP 的高层 API 设计可以继续沿用
