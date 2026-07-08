cmake_minimum_required(VERSION 3.16)
project(luxitech_adj_visualization LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

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
find_library(OpenCV_HIGHGUI_LIB
    NAMES opencv_highgui libopencv_highgui.so.4.5d libopencv_highgui.so.4.5.4d
    PATHS /usr/lib/aarch64-linux-gnu /usr/local/lib /usr/lib
    REQUIRED)

set(OpenCV_LIBS
    ${OpenCV_HIGHGUI_LIB}
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

add_library(luxitech_mvs STATIC
    ../bsp/src/mvs_camera.cpp
)

target_include_directories(luxitech_mvs
    PUBLIC
        ../../../bsp/include
        ${OpenCV_INCLUDE_DIR}
)

target_link_libraries(luxitech_mvs
    PUBLIC
        ${OpenCV_LIBS}
        ${MVS_CAMERA_CONTROL_LIB}
)

add_executable(02-adj_visualization
    main.cpp
)

target_link_libraries(02-adj_visualization
    PRIVATE
        luxitech_mvs
)

set_target_properties(02-adj_visualization PROPERTIES
    BUILD_RPATH "${MVS_CAMERA_CONTROL_DIR}"
)
