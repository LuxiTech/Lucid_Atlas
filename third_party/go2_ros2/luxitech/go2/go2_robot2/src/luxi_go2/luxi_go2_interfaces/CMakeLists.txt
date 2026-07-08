cmake_minimum_required(VERSION 3.8)
project(luxi_go2_interfaces)

find_package(ament_cmake REQUIRED)

install(DIRECTORY msg srv action DESTINATION share/${PROJECT_NAME})

ament_package()
