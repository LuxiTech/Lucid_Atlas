cmake_minimum_required(VERSION 3.8)
project(luxi_web_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(fast_lio REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)

add_executable(network_status_node src/network_status_node.cpp)
ament_target_dependencies(network_status_node rclcpp std_msgs)

add_executable(command_guard_node src/command_guard_node.cpp)
ament_target_dependencies(command_guard_node rclcpp std_msgs geometry_msgs)

add_executable(demo_state_node src/demo_state_node.cpp)
ament_target_dependencies(demo_state_node rclcpp std_msgs geometry_msgs)

add_executable(web_api_node src/web_api_node.cpp)
target_compile_features(web_api_node PUBLIC cxx_std_17)
ament_target_dependencies(web_api_node rclcpp rcl_interfaces std_msgs geometry_msgs sensor_msgs nav_msgs fast_lio)
target_link_libraries(web_api_node Boost::system ${OpenCV_LIBS})

install(TARGETS
  network_status_node
  command_guard_node
  demo_state_node
  web_api_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config web docs scripts
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

ament_package()
