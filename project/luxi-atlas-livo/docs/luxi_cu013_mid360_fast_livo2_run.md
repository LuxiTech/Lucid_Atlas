# Luxi CU013 + MID360 FAST-LIVO2 Run

## 1. Build

```bash
cd /home/nvidia/project/luxi-atlas

source /opt/ros/humble/setup.bash
source /home/nvidia/project/luxi-atlas/device/mid360/install/setup.bash
source /home/nvidia/project/luxi-atlas/device/luxi-hikrobot/ROS2/MV-CU013-A0UC/ws/install/setup.bash

colcon build --symlink-install --base-paths third_party/rpg_vikit project/luxi-atlas-livo
source /home/nvidia/project/luxi-atlas/install/setup.bash
```

## 2. Start Camera And MID360

Terminal 1:

```bash
cd /home/nvidia/project/luxi-atlas

source /opt/ros/humble/setup.bash
source /home/nvidia/project/luxi-atlas/device/mid360/install/setup.bash
source /home/nvidia/project/luxi-atlas/device/luxi-hikrobot/ROS2/MV-CU013-A0UC/ws/install/setup.bash
source /home/nvidia/project/luxi-atlas/install/setup.bash

ros2 launch fast_livo luxi_device_cu013_mid360_preview.launch.py \
  mid360_xfer_format:=1 \
  external_trigger_enabled:=true \
  trigger_source:=Line0 \
  trigger_activation:=RisingEdge \
  use_rviz:=false
```

Expected MID360 log:

```text
livox time_type: 2
livox/lidar publish use livox custom format
```

Do not split `ros2 launch fast_livo` and the launch file into two commands.
The following form is wrong:

```bash
ros2 launch fast_livo

luxi_device_cu013_mid360_preview.launch.py ...
```

Use one continuous command, or keep the trailing `\` exactly as shown above.
If this device launch is not running correctly, RViz will not show the mapping
result because FAST-LIVO2 has no camera, LiDAR, or IMU input.

## 3. Check Device Topics

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/project/luxi-atlas/install/setup.bash

ros2 node list | grep -E 'hikrobot|livox'
ros2 topic list | grep -E 'hikrobot|livox'
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic hz /hikrobot/cu013/rgb_img
```

Expected:

```text
/livox/lidar around 10 Hz
/livox/imu around 200 Hz
/hikrobot/cu013/rgb_img around 10 Hz
```

If `/livox/lidar`, `/livox/imu`, or `/hikrobot/cu013/rgb_img` is missing, do
not start mapping yet. Fix Terminal 1 first.

## 4. Tune Camera Before Mapping

Keep Terminal 1 running. The ROS2 CU013 node exposes the same basic camera
controls used by the `02-adj_visualization` sample, so tune the camera directly
from another terminal with `ros2 param set`.

Recommended starting point for FAST-LIVO2:

```text
ExposureAuto -> exposure_auto = Continuous
GainAuto -> gain_auto = Off
Gain -> gain = 15.0
```

Apply it:

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/project/luxi-atlas/device/luxi-hikrobot/ROS2/MV-CU013-A0UC/ws/install/setup.bash

ros2 param set /hikrobot/cu013/hikrobot_cu013_camera_node exposure_auto '"Continuous"'
ros2 param set /hikrobot/cu013/hikrobot_cu013_camera_node gain_auto '"Off"'
ros2 param set /hikrobot/cu013/hikrobot_cu013_camera_node gain 15.0
```

If auto exposure is not bright enough in the current lighting, switch to manual
exposure and increase exposure time:

```bash
ros2 param set /hikrobot/cu013/hikrobot_cu013_camera_node exposure_time_us 40000.0
```

Increase or decrease manual exposure in small steps:

```bash
ros2 param set /hikrobot/cu013/hikrobot_cu013_camera_node exposure_time_us 30000.0
ros2 param set /hikrobot/cu013/hikrobot_cu013_camera_node exposure_time_us 50000.0
ros2 param set /hikrobot/cu013/hikrobot_cu013_camera_node exposure_time_us 80000.0
```

Keep `gain_auto` off; adjust manual gain only if exposure time is already too
long:

```bash
ros2 param set /hikrobot/cu013/hikrobot_cu013_camera_node gain_auto '"Off"'
ros2 param set /hikrobot/cu013/hikrobot_cu013_camera_node gain 12.0
ros2 param set /hikrobot/cu013/hikrobot_cu013_camera_node gain 18.0
```

Other `02-adj_visualization` control-name mappings:

```text
ExposureTime -> exposure_time_us
PixelFormat -> pixel_format
Width -> width
Height -> height
OffsetX -> offset_x
OffsetY -> offset_y
TriggerMode Off -> external_trigger_enabled false
TriggerMode On + TriggerSource Line0 + TriggerActivation RisingEdge:
  external_trigger_enabled true
  trigger_source "Line0"
  trigger_activation "RisingEdge"
```

Examples:

```bash
ros2 param set /hikrobot/cu013/hikrobot_cu013_camera_node pixel_format '"BayerGB8"'
ros2 param set /hikrobot/cu013/hikrobot_cu013_camera_node external_trigger_enabled true
ros2 param set /hikrobot/cu013/hikrobot_cu013_camera_node trigger_source '"Line0"'
ros2 param set /hikrobot/cu013/hikrobot_cu013_camera_node trigger_activation '"RisingEdge"'
```

Check the active ROS parameters:

```bash
ros2 param get /hikrobot/cu013/hikrobot_cu013_camera_node exposure_auto
ros2 param get /hikrobot/cu013/hikrobot_cu013_camera_node exposure_time_us
ros2 param get /hikrobot/cu013/hikrobot_cu013_camera_node gain_auto
ros2 param get /hikrobot/cu013/hikrobot_cu013_camera_node gain
ros2 param get /hikrobot/cu013/hikrobot_cu013_camera_node external_trigger_enabled
```

Use quoted strings for enum parameters such as `Off`, `Line0`, and
`RisingEdge`; otherwise ROS 2 CLI may parse `Off` as a boolean value.

`AutoTargetBrightness` and `AutoExposureTimeUpperLimit` are not exposed through
the ROS2 node on this setup because the CU013 returns MVS write error
`0x80000109` for those nodes while opened through the ROS camera driver.

## 5. Start FAST-LIVO2 Mapping

Terminal 3:

```bash
cd /home/nvidia/project/luxi-atlas

source /opt/ros/humble/setup.bash
source /home/nvidia/project/luxi-atlas/install/setup.bash

ros2 launch fast_livo mapping_luxi_cu013_mid360.launch.py use_rviz:=true
```

After startup, the mapping terminal should print messages similar to:

```text
scale: 0.5
intrinsic: ...
FIRST LIDAR FRAME!
Gravity Alignment Finished
```

Terminal 2 can be used to check that FAST-LIVO2 is publishing the map:

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/project/luxi-atlas/install/setup.bash

ros2 node list | grep -E 'laserMapping|rviz'
ros2 topic list | grep -E 'cloud_registered|Laser_map|path'
ros2 topic hz /cloud_registered
ros2 topic echo --once /cloud_registered --field header
```

If `/cloud_registered` has no frequency, the mapping node is not receiving
synchronized LiDAR/IMU/RGB data. Check Terminal 1 device topics first, then
check Terminal 3 for errors such as camera model, `libusb_set_option`, or time
sync warnings.

The mapping launch loads:

```text
config/luxi/luxi_cu013_mid360.yaml
config/luxi/camera_cu013.yaml
```

The mapping launch also sets:

```text
LD_PRELOAD=/lib/aarch64-linux-gnu/libusb-1.0.so.0
```

and removes MVS SDK library paths from `LD_LIBRARY_PATH` for the mapping
process. This prevents PCL from loading the Hikrobot MVS SDK copy of
`libusb-1.0.so.0`.

Current Luxi config also enables:

```text
time_offset.auto_img_time_offset: true
publish.image_skip_num: 2
pcd_save.pcd_save_en: true
pcd_save.interval: -1
```

Because the CU013 image header uses host time while MID360 uses the MCU
simulated PPS/GPS time base, FAST-LIVO2 calibrates `img_time_offset` once on
each mapping process startup. The mapping terminal should print:

```text
Auto image time offset calibrated: ...
```

The camera still publishes about 10 Hz, but FAST-LIVO2 only consumes every
second RGB frame. The color update rate is therefore about 5 Hz, which reduces
CPU pressure on Jetson while keeping the LiDAR/IMU mapping stream continuous.

## 6. Stop And Save Map

When mapping is finished, press `Ctrl-C` once in Terminal 3 and wait for
`fastlivo_mapping` to exit cleanly. With `pcd_save.interval: -1`, FAST-LIVO2
saves the accumulated map during shutdown.

The saved files are named with the current system time:

```text
/home/nvidia/project/luxi-atlas/project/luxi-atlas-livo/Log/PCD/all_raw_points_YYYYMMDD_HHMMSS.pcd
/home/nvidia/project/luxi-atlas/project/luxi-atlas-livo/Log/PCD/all_downsampled_points_YYYYMMDD_HHMMSS.pcd
```

`all_raw_points_*` is the full colored point cloud. `all_downsampled_points_*`
is voxel filtered by `pcd_save.filter_size_pcd` and is faster to open for
inspection.

List saved maps:

```bash
ls -lh /home/nvidia/project/luxi-atlas/project/luxi-atlas-livo/Log/PCD/*_points_*.pcd
```

Check the newest downsampled map header and point count:

```bash
MAP=$(ls -t /home/nvidia/project/luxi-atlas/project/luxi-atlas-livo/Log/PCD/all_downsampled_points_*.pcd | head -1)
sed -n '1,11p' "$MAP"
```

## 7. View Saved Map

Open the newest downsampled map:

```bash
MAP=$(ls -t /home/nvidia/project/luxi-atlas/project/luxi-atlas-livo/Log/PCD/all_downsampled_points_*.pcd | head -1)

env LD_PRELOAD=/lib/aarch64-linux-gnu/libusb-1.0.so.0 \
  LD_LIBRARY_PATH="/lib/aarch64-linux-gnu:/usr/lib/aarch64-linux-gnu:$(printf '%s' "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v 'MvCamCtrlSDK/lib/aarch64' | paste -sd: -)" \
  pcl_viewer "$MAP"
```

Open the newest raw map:

```bash
MAP=$(ls -t /home/nvidia/project/luxi-atlas/project/luxi-atlas-livo/Log/PCD/all_raw_points_*.pcd | head -1)

env LD_PRELOAD=/lib/aarch64-linux-gnu/libusb-1.0.so.0 \
  LD_LIBRARY_PATH="/lib/aarch64-linux-gnu:/usr/lib/aarch64-linux-gnu:$(printf '%s' "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v 'MvCamCtrlSDK/lib/aarch64' | paste -sd: -)" \
  pcl_viewer "$MAP"
```

If `pcl_viewer` reports `undefined symbol: libusb_set_option`, use the `env`
command above. The cause is that the Hikrobot MVS SDK library path can shadow
the system `libusb`, while PCL needs the system version.

## 8. RViz Has No Map

If RViz opens but does not show building content after boot, check in this
order:

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/project/luxi-atlas/install/setup.bash

ros2 node list
ros2 topic list | grep -E 'livox|hikrobot|cloud_registered'
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic hz /hikrobot/cu013/rgb_img
ros2 topic hz /cloud_registered
```

Common causes:

- The device command was split after `ros2 launch fast_livo`, so CU013 and
  MID360 were never started.
- Terminal 1 was closed. The mapping launch does not start the camera and
  MID360; it only starts FAST-LIVO2 and RViz.
- Mapping was started before `/livox/lidar`, `/livox/imu`, and
  `/hikrobot/cu013/rgb_img` were publishing.
- `fastlivo_mapping` exited. Check Terminal 3; RViz can remain open even after
  the mapping node has died.
- RViz fixed frame or display was changed. Use the checked-in config
  `rviz_cfg/fast_livo2.rviz`, where `/cloud_registered` is enabled and the
  fixed frame is `camera_init`.

## Notes

- Do not use `mid360_xfer_format:=0` for FAST-LIVO2 mapping. That mode publishes `sensor_msgs/msg/PointCloud2` and is only for RViz preview.
- If `MV_CC_OpenDevice failed: 0x80000203` appears, another camera process is probably occupying the CU013. Stop the old process before restarting.
- `time_offset.auto_img_time_offset: true` is required for the current MCU simulated time setup. Do not hard-code `img_time_offset` unless this auto mode is disabled.
- If `/cloud_registered` does not publish after boot, check the mapping terminal for `Auto image time offset calibrated`. If that line is missing, FAST-LIVO2 has not received both LiDAR and camera frames.
- RViz is configured to keep `/cloud_registered` for a long decay time, so the live view looks like an accumulated map. The actual persisted result is the PCD file saved on shutdown.
- If logs keep showing `imu time stamp Jumps` or `Self sync IMU and LiDAR` during long runs, check the MID360 PPS/GPS timestamp input. The node can run with `common.ros_driver_bug_fix: true`, and the local FAST-LIVO2 build resets the IMU buffer when a jump is close to the current LiDAR timestamp. The stable fix is still to keep LiDAR point cloud and IMU timestamps on one hardware time base.
