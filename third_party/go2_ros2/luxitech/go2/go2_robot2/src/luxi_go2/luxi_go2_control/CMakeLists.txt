cmake_minimum_required(VERSION 3.8)
project(luxi_go2_control)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config include DESTINATION share/${PROJECT_NAME})

ament_package()
