cmake_minimum_required(VERSION 3.10)
project(pcd_to_octomap)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(octomap REQUIRED)

add_executable(pcd2octomap src/pcd2octomap.cpp)
add_executable(inspect_octomap src/inspect_octomap.cpp)

target_include_directories(pcd2octomap PRIVATE
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

target_link_libraries(pcd2octomap
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  usb-1.0
)

target_compile_definitions(pcd2octomap PRIVATE ${PCL_DEFINITIONS})

target_include_directories(inspect_octomap PRIVATE ${OCTOMAP_INCLUDE_DIRS})
target_link_libraries(inspect_octomap ${OCTOMAP_LIBRARIES})

