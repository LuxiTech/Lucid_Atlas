cmake_minimum_required(VERSION 3.8)
project(go2_field_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(
  PROGRAMS
    scripts/initial_pose_publisher.py
    scripts/global_localization_trigger.py
    scripts/safety_gate.py
    scripts/cmd_vel_watchdog.py
    scripts/fake_odom_publisher.py
    scripts/official_cmd_vel_bridge.py
    scripts/sport_state_bridge.py
    scripts/lowstate_joint_state_bridge.py
    scripts/pointcloud_relay.py
    scripts/front_video_bridge.py
    scripts/keyboard_teleop.py
    scripts/scan_boundary_filter.py
    scripts/scan_tuning_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
