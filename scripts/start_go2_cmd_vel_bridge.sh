#!/usr/bin/env bash
set -eo pipefail

cd /home/nvidia/project/luxi-atlas

source /home/nvidia/project/luxi-atlas/third_party/go2_ros2/setup.sh
source /home/nvidia/project/luxi-atlas/third_party/go2_ros2/luxitech/go2/go2_robot2/install/setup.bash

publish_request() {
  local topic="$1"
  local request_id="$2"
  local api_id="$3"
  local parameter="$4"

  timeout 8 ros2 topic pub --qos-reliability best_effort --once "${topic}" \
    unitree_api/msg/Request \
    "{header: {identity: {id: ${request_id}, api_id: ${api_id}}, lease: {id: 0}, policy: {priority: 0, noreply: false}}, parameter: '${parameter}', binary: []}" \
    >/dev/null 2>&1 || true
}

# Go2 may boot in an occupied motion mode such as mcf. Release it and enable the
# official sport service before accepting navigation cmd_vel commands.
publish_request /api/motion_switcher/request 9101 1003 ''
sleep 0.5
publish_request /api/robot_state/request 9102 1001 '{"name":"sport_mode","switch":1}'
sleep 0.5
publish_request /api/sport/request 9103 1002 ''
sleep 0.5

exec ros2 launch go2_cmd_vel_bridge go2_cmd_vel_bridge.launch.py \
  cmd_vel_topic:=/cmd_vel \
  request_topic:=/api/sport/request \
  expected_sender_ip:=192.168.123.51 \
  max_linear_x:=0.5 \
  max_linear_y:=0.5 \
  max_angular_z:=1.0 \
  cmd_timeout_sec:=0.2 \
  stop_burst_count:=12 \
  qos_depth:=1 \
  desired_motion_mode:=1 \
  desired_gait_type:=1 \
  enforce_desired_motion_mode:=true
