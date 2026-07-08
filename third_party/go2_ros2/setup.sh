#!/bin/bash
echo "Setup unitree ros2 environment"

source /opt/ros/humble/setup.bash
source /home/nvidia/project/luxi-atlas/third_party/go2_ros2/cyclonedds_ws/install/setup.bash

LOCAL_ROS_DEB_PREFIX=/home/nvidia/project/luxi-atlas/.local_ros_debs/root/opt/ros/humble
if [ -d "${LOCAL_ROS_DEB_PREFIX}" ]; then
  export AMENT_PREFIX_PATH="${LOCAL_ROS_DEB_PREFIX}:${AMENT_PREFIX_PATH:-}"
  export LD_LIBRARY_PATH="${LOCAL_ROS_DEB_PREFIX}/lib:${LOCAL_ROS_DEB_PREFIX}/lib/aarch64-linux-gnu:${LD_LIBRARY_PATH:-}"
  export CMAKE_PREFIX_PATH="${LOCAL_ROS_DEB_PREFIX}:${CMAKE_PREFIX_PATH:-}"
fi

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enP8p1s0" priority="default" multicast="default" /></Interfaces></General></Domain></CycloneDDS>'

echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
echo "CycloneDDS interface=enP8p1s0"
