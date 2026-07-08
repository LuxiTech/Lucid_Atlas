#!/usr/bin/env bash
set -euo pipefail

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

for scene in scene01 scene02 scene03; do
  test -f "$DATA/$scene.bag"
  test -f "$DATA/$scene.jpg"

  rosparam delete / 2>/dev/null || true
  rosparam load "$PKG_SRC/config/qr_params.yaml"
  rosparam set /bag_path "$DATA/$scene.bag"
  rosparam set /image_path "$DATA/$scene.jpg"
  rosparam set /output_path "$OUTPUT"

  echo "Running single-scene calibration: $scene"
  set +e
  timeout 8s rosrun fast_calib fast_calib
  STATUS=$?
  set -e
  if [[ "$STATUS" -ne 0 && "$STATUS" -ne 124 ]]; then
    exit "$STATUS"
  fi
done

rosparam delete / 2>/dev/null || true
rosparam load "$PKG_SRC/config/qr_params.yaml"
rosparam set /output_path "$OUTPUT"

echo "Running multi-scene calibration"
rosrun fast_calib multi_fast_calib

echo "Result:"
cat "$OUTPUT/multi_calib_result.txt"
