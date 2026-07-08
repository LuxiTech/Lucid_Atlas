cmake_minimum_required(VERSION 3.10)
project(pcd_to_pgm)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL REQUIRED COMPONENTS common io filters)

add_executable(pcd2pgm src/pcd_to_pgm.cpp)

target_include_directories(pcd2pgm PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(pcd2pgm ${PCL_LIBRARIES} usb-1.0)
target_compile_definitions(pcd2pgm PRIVATE ${PCL_DEFINITIONS})
