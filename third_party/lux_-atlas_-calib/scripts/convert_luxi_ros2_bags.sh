#!/usr/bin/env bash
set -euo pipefail

DATA=/home/nvidia/project/luxi-atlas/third_party/lux_-atlas_-calib/calib_data/luxi_cu013_mid360
mkdir -p "$DATA"

for n in 01 02 03; do
  SRC="/host_tmp/luxi_scene${n}_ros2"
  DST="$DATA/scene${n}.bag"
  test -d "$SRC"
  rm -rf "$DST"
  rosbags-convert --dst "$DST" "$SRC"
done

ls -lh "$DATA"/scene*.bag "$DATA"/scene*.jpg
