# MV-CU013-A0UC ROS2 Camera Sample

This package publishes the Hikrobot MV-CU013-A0UC camera image as ROS2 topics for RViz.

## Topics

- `/hikrobot/cu013/image_raw` (`sensor_msgs/msg/Image`, `bgr8`)
- `/hikrobot/cu013/rgb_img` (`sensor_msgs/msg/Image`, `bgr8`, image_transport-compatible raw topic)
- `/hikrobot/cu013/camera_info` (`sensor_msgs/msg/CameraInfo`)

## Build

```bash
cd /home/nvidia/project/luxi-atlas/device/luxi-hikrobot/ROS2/MV-CU013-A0UC/ws
source /opt/ros/humble/setup.bash
source /home/nvidia/project/luxi-atlas/device/luxi-hikrobot/SDK/extracted/MvCamCtrlSDK_Runtime-4.8.0_aarch64_20260512/set_env_path.sh \
  /home/nvidia/project/luxi-atlas/device/luxi-hikrobot/SDK/runtime/MvCamCtrlSDK 4.8.0
colcon build --symlink-install --cmake-args \
  -DMVS_SDK_PATH=/home/nvidia/project/luxi-atlas/device/luxi-hikrobot/SDK/runtime/MvCamCtrlSDK
```

## Run

```bash
cd /home/nvidia/project/luxi-atlas/device/luxi-hikrobot/ROS2/MV-CU013-A0UC/ws
source /opt/ros/humble/setup.bash
source install/setup.bash
source /home/nvidia/project/luxi-atlas/device/luxi-hikrobot/SDK/extracted/MvCamCtrlSDK_Runtime-4.8.0_aarch64_20260512/set_env_path.sh \
  /home/nvidia/project/luxi-atlas/device/luxi-hikrobot/SDK/runtime/MvCamCtrlSDK 4.8.0
ros2 launch luxi_hikrobot_cu013_camera cu013.launch.py
```

RViz opens with the Image display subscribed to `/hikrobot/cu013/rgb_img`.
If the Image panel is blank, select `/hikrobot/cu013/rgb_img` manually in the Image display and keep Transport Hint as `raw`.

## Check Topics

```bash
ros2 topic list | grep hikrobot
ros2 topic hz /hikrobot/cu013/rgb_img
ros2 topic echo /hikrobot/cu013/rgb_img/header --once
```

## Runtime Parameters

```bash
ros2 param set /hikrobot/cu013/hikrobot_cu013_camera_node exposure_auto '"Continuous"'
ros2 param set /hikrobot/cu013/hikrobot_cu013_camera_node gain_auto '"Off"'
ros2 param set /hikrobot/cu013/hikrobot_cu013_camera_node gain 15.0

# Manual exposure fallback if auto exposure is still too dark.
ros2 param set /hikrobot/cu013/hikrobot_cu013_camera_node exposure_auto '"Off"'
ros2 param set /hikrobot/cu013/hikrobot_cu013_camera_node exposure_time_us 40000.0

# 02-adj_visualization name mapping:
# ExposureTime -> exposure_time_us
# PixelFormat -> pixel_format
# Width -> width
# Height -> height
# OffsetX -> offset_x
# OffsetY -> offset_y
```

## External PWM Trigger

Edit `config/cu013.yaml`:

```yaml
external_trigger_enabled: true
trigger_source: "Line0"
trigger_activation: "RisingEdge"
```

Runtime switch:

```bash
ros2 param set /hikrobot/cu013/hikrobot_cu013_camera_node external_trigger_enabled true
ros2 param set /hikrobot/cu013/hikrobot_cu013_camera_node trigger_source '"Line0"'
ros2 param set /hikrobot/cu013/hikrobot_cu013_camera_node trigger_activation '"RisingEdge"'
```

Launch-time test:

```bash
ros2 launch luxi_hikrobot_cu013_camera cu013.launch.py \
  external_trigger_enabled:=true \
  trigger_source:=Line0 \
  trigger_activation:=RisingEdge
```

When external trigger is enabled, image publishing follows the incoming PWM trigger rate.
