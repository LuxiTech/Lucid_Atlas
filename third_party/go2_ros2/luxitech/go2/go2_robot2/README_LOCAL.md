# GO2 Robot2 Workspace

This workspace contains a CHAMP-based Gazebo simulation stack adapted from:

- `https://github.com/anujjain-dev/unitree-go2-ros2`

It is kept separate from `go2_robot` on purpose because both workspaces install
packages such as `go2_description`, and sourcing both overlays together will
cause package resolution conflicts.

## Workspace Layout

- `go2_robot`: Unitree-oriented workspace used for the official model, driver,
  and RViz-only visualization flow.
- `go2_robot2`: CHAMP + Gazebo simulation workspace for future simulation-based
  development.

## Build

```bash
source /opt/ros/humble/setup.bash
cd /root/ws/unitree_ros2/luxitech/go2/go2_robot2
colcon build --symlink-install
```

## Use This Workspace

```bash
source /opt/ros/humble/setup.bash
source /root/ws/unitree_ros2/luxitech/go2/go2_robot2/install/setup.bash
```

Do not source `go2_robot/install/setup.bash` in the same shell when you want to
run the Gazebo simulation from `go2_robot2`.

## Verified Commands

Start Gazebo:

```bash
ros2 launch go2_config gazebo.launch.py
```

Start Gazebo without the client window:

```bash
ros2 launch go2_config gazebo.launch.py gui:=false rviz:=false
```

Start Gazebo with RViz:

```bash
ros2 launch go2_config gazebo.launch.py rviz:=true
```

Start the Velodyne variant:

```bash
ros2 launch go2_config gazebo_velodyne.launch.py
```

## Current Status

- Gazebo spawn works in this environment.
- `gazebo_ros2_control` and the CHAMP controller stack load successfully.
- Headless launch is adapted so `gui:=false` now suppresses `gzclient`.
- Upstream still marks SLAM and Nav2 as not fully working, so treat those as
  starting points for development rather than production-ready demos.
