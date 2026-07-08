cmake_minimum_required(VERSION 3.8)
project(go2_field_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config maps rviz DESTINATION share/${PROJECT_NAME})

ament_package()

