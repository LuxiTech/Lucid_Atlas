cmake_minimum_required(VERSION 3.8)
project(mapping_lab_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(
  PROGRAMS
    scripts/initial_pose_publisher.py
    scripts/global_localization_trigger.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
