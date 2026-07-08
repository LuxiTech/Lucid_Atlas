# Reboot Startup And Validation

Use this launch after reboot when the phone web page must show RGB, map,
live cloud, localization, and web driving control:

```bash
cd /home/nvidia/project/luxi-atlas
source install/setup.bash
ros2 launch luxi_web_control web_navigation.launch.py
```

The web-only launch below starts only the control API and static page. It does
not start camera, LiDAR, FAST-LIO, grid map, or localization, so RGB/map panels
will stay in waiting state:

```bash
ros2 launch luxi_web_control web_control.launch.py
```

After startup, validate from terminal:

```bash
/home/nvidia/project/luxi-atlas/install/luxi_web_control/share/luxi_web_control/scripts/check_web_runtime.sh
```

Override IP if the WiFi address changes:

```bash
LUXI_WEB_HOST=<device_wifi_ip> \
/home/nvidia/project/luxi-atlas/install/luxi_web_control/share/luxi_web_control/scripts/check_web_runtime.sh
```

The phone URL is:

```text
http://<device_wifi_ip>:8080
```
