cmake_minimum_required(VERSION 3.8)
project(luxi_go2_utils)

find_package(ament_cmake REQUIRED)

install(DIRECTORY scripts DESTINATION share/${PROJECT_NAME})

ament_package()
