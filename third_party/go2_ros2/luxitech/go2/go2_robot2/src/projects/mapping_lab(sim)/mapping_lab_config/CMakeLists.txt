cmake_minimum_required(VERSION 3.8)
project(mapping_lab_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY rviz config maps worlds DESTINATION share/${PROJECT_NAME})

ament_package()
