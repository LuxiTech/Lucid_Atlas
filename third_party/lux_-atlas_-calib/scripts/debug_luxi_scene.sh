#!/usr/bin/env bash
set -euo pipefail

SCENE="${1:-scene01}"
PROJECT_ROOT=/home/nvidia/project/luxi-atlas
PKG_SRC=$PROJECT_ROOT/third_party/lux_-atlas_-calib
WS=/catkin_ws
DATA=$PKG_SRC/calib_data/luxi_cu013_mid360
OUTPUT=$PKG_SRC/output/luxi_cu013_mid360

source /opt/ros/noetic/setup.bash

mkdir -p "$WS/src" "$OUTPUT"
ln -sfn "$PKG_SRC" "$WS/src/fast_calib"

cd "$WS"
catkin build fast_calib
source "$WS/devel/setup.bash"

roscore >/tmp/luxi_fast_calib_roscore.log 2>&1 &
ROSCORE_PID=$!
trap 'kill "$ROSCORE_PID" 2>/dev/null || true' EXIT
sleep 2

rosparam delete / 2>/dev/null || true
rosparam load "$PKG_SRC/config/qr_params.yaml"
rosparam set /bag_path "$DATA/$SCENE.bag"
rosparam set /image_path "$DATA/$SCENE.jpg"
rosparam set /output_path "$OUTPUT"

echo "Running $SCENE debug publisher. Open rviz and inspect /filtered_cloud, /plane_cloud, /edge_cloud, /center_cloud."
rosrun fast_calib fast_calib
