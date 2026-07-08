cmake_minimum_required(VERSION 3.10)
project(octomap_to_occupancygrid)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(octomap REQUIRED)

add_executable(octomap2grid src/octomap_to_occupancygrid.cpp)

target_include_directories(octomap2grid PRIVATE ${OCTOMAP_INCLUDE_DIRS})
target_link_libraries(octomap2grid ${OCTOMAP_LIBRARIES})
