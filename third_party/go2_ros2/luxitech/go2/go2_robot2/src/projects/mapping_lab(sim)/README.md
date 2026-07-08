# mapping_lab

This directory contains project-specific packages for the mapping lab workflow.

Third-party map assets are stored at:

`/root/ws/unitree_ros2/luxitech/go2/go2_robot2/src/third_party/turtlebot3_simulations`

## Quick Start (GO2 in TurtleBot3 World)

```bash
source /opt/ros/humble/setup.bash
source /root/ws/unitree_ros2/luxitech/go2/go2_robot2/install/setup.bash
ros2 launch mapping_lab_bringup sim_lidar_view.launch.py gui:=true
```

## SLAM Toolbox Mapping

```bash
source /opt/ros/humble/setup.bash
source /root/ws/unitree_ros2/luxitech/go2/go2_robot2/install/setup.bash
ros2 launch mapping_lab_bringup sim_slam_toolbox.launch.py gui:=true
```

## Nav2 Localization + Navigation

```bash
source /opt/ros/humble/setup.bash
source /root/ws/unitree_ros2/luxitech/go2/go2_robot2/install/setup.bash
ros2 launch mapping_lab_bringup sim_nav2.launch.py gui:=true
```

Navigation default map:
`/root/ws/unitree_ros2/luxitech/go2/go2_robot2/src/projects/mapping_lab/mapping_lab_config/maps/lab_map.yaml`

Default spawn in `turtlebot3_house_classic.world` is set to a stable area:
`world_init_x:=1.5 world_init_y:=-2.0 world_init_z:=0.35`

If you need to override it:

```bash
ros2 launch mapping_lab_bringup sim_lidar_view.launch.py gui:=true \
  world_init_x:=1.5 world_init_y:=-2.0 world_init_z:=0.35
```

Keyboard control (same as before):

```bash
source /opt/ros/humble/setup.bash
source /root/ws/unitree_ros2/luxitech/go2/go2_robot2/install/setup.bash
ros2 run champ_teleop champ_teleop.py
```

If you want to switch to another TurtleBot3 world:

```bash
ros2 launch mapping_lab_bringup sim_lidar_view.launch.py \
  world:=/root/ws/unitree_ros2/luxitech/go2/go2_robot2/src/projects/mapping_lab/mapping_lab_config/worlds/turtlebot3_house_classic.world
```

## Gazebo GUI Stability Note

- Do **not** export `GAZEBO_RESOURCE_PATH` to `.../turtlebot3_gazebo`.
- This path causes `gzclient` to crash with:
  `Assertion 'px != 0' failed (gazebo::rendering::Camera*)`.
- `mapping_lab_bringup` only appends `GAZEBO_MODEL_PATH` for TurtleBot3 map models.
