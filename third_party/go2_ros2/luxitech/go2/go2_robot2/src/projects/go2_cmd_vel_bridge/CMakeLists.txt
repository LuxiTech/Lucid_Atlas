cmake_minimum_required(VERSION 3.8)
project(go2_cmd_vel_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(unitree_api REQUIRED)
find_package(unitree_go REQUIRED)

add_library(${PROJECT_NAME}_adapter src/cmd_vel_adapter.cpp)
target_include_directories(${PROJECT_NAME}_adapter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME}_adapter PUBLIC cxx_std_17)
ament_target_dependencies(${PROJECT_NAME}_adapter
  geometry_msgs
  unitree_api)

add_executable(go2_cmd_vel_bridge_node src/go2_cmd_vel_bridge_node.cpp)
target_link_libraries(go2_cmd_vel_bridge_node ${PROJECT_NAME}_adapter)
target_compile_features(go2_cmd_vel_bridge_node PUBLIC cxx_std_17)
ament_target_dependencies(go2_cmd_vel_bridge_node
  geometry_msgs
  rclcpp
  unitree_api
  unitree_go)

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}_adapter
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS go2_cmd_vel_bridge_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_cmd_vel_adapter test/test_cmd_vel_adapter.cpp)
  if(TARGET test_cmd_vel_adapter)
    target_link_libraries(test_cmd_vel_adapter ${PROJECT_NAME}_adapter)
    ament_target_dependencies(test_cmd_vel_adapter
      geometry_msgs
      unitree_api)
  endif()
endif()

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(geometry_msgs unitree_api unitree_go)
ament_package()
