cmake_minimum_required(VERSION 3.16)
project(luxi_hikrobot_cu013_camera LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)

find_path(OpenCV_INCLUDE_DIR
    NAMES opencv2/core.hpp
    PATHS /usr/include/opencv4 /usr/local/include/opencv4
    REQUIRED)

find_library(OpenCV_CORE_LIB
    NAMES opencv_core libopencv_core.so.4.5d libopencv_core.so.4.5.4d
    PATHS /usr/lib/aarch64-linux-gnu /usr/local/lib /usr/lib
    REQUIRED)
find_library(OpenCV_IMGPROC_LIB
    NAMES opencv_imgproc libopencv_imgproc.so.4.5d libopencv_imgproc.so.4.5.4d
    PATHS /usr/lib/aarch64-linux-gnu /usr/local/lib /usr/lib
    REQUIRED)
find_library(OpenCV_IMGCODECS_LIB
    NAMES opencv_imgcodecs libopencv_imgcodecs.so.4.5d libopencv_imgcodecs.so.4.5.4d
    PATHS /usr/lib/aarch64-linux-gnu /usr/local/lib /usr/lib
    REQUIRED)

set(OpenCV_LIBS
    ${OpenCV_IMGCODECS_LIB}
    ${OpenCV_IMGPROC_LIB}
    ${OpenCV_CORE_LIB})

if(NOT MVS_SDK_PATH)
    set(MVS_SDK_PATH "$ENV{MVCAM_SDK_PATH}")
    if(NOT MVS_SDK_PATH)
        set(MVS_SDK_PATH "/opt/MVS")
    endif()
endif()

find_library(MVS_CAMERA_CONTROL_LIB
    NAMES MvCameraControl libMvCameraControl.so.4.8.0.3 libMvCameraControl.so.4.7.0.1
    PATHS
        "${MVS_SDK_PATH}/lib/aarch64"
        "${MVS_SDK_PATH}/lib/armhf"
        "${MVS_SDK_PATH}/lib/64"
    NO_DEFAULT_PATH
    REQUIRED)

get_filename_component(MVS_CAMERA_CONTROL_DIR "${MVS_CAMERA_CONTROL_LIB}" DIRECTORY)
get_filename_component(HIKROBOT_REPO_ROOT
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../../.."
    ABSOLUTE)

add_library(luxitech_mvs_cu013 STATIC
    "${HIKROBOT_REPO_ROOT}/Project/MV-CU013-A0UC/bsp/src/mvs_camera.cpp")

target_include_directories(luxitech_mvs_cu013
    PUBLIC
        "${HIKROBOT_REPO_ROOT}/bsp/include"
        ${OpenCV_INCLUDE_DIR})

target_link_libraries(luxitech_mvs_cu013
    PUBLIC
        ${OpenCV_LIBS}
        ${MVS_CAMERA_CONTROL_LIB})

add_executable(hikrobot_cu013_camera_node
    src/hikrobot_cu013_camera_node.cpp)

target_link_libraries(hikrobot_cu013_camera_node
    luxitech_mvs_cu013)

ament_target_dependencies(hikrobot_cu013_camera_node
    rclcpp
    sensor_msgs
    image_transport)

set_target_properties(hikrobot_cu013_camera_node PROPERTIES
    BUILD_RPATH "${MVS_CAMERA_CONTROL_DIR}"
    INSTALL_RPATH "${MVS_CAMERA_CONTROL_DIR}")

install(TARGETS hikrobot_cu013_camera_node
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch rviz
    DESTINATION share/${PROJECT_NAME})

ament_package()
