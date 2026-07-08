cmake_minimum_required(VERSION 3.16)
project(luxitech_mini_visualization LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED)

set(MVS_SDK_PATH "$ENV{MVCAM_SDK_PATH}")
if(NOT MVS_SDK_PATH)
    set(MVS_SDK_PATH "/opt/MVS")
endif()

find_library(MVS_CAMERA_CONTROL_LIB
    NAMES MvCameraControl
    PATHS "${MVS_SDK_PATH}/lib/64"
    NO_DEFAULT_PATH
    REQUIRED)

add_library(luxitech_mvs STATIC
    ../../bsp/src/mvs_camera.cpp
)

target_include_directories(luxitech_mvs
    PUBLIC
        ../../bsp/include
)

target_link_libraries(luxitech_mvs
    PUBLIC
        ${OpenCV_LIBS}
        ${MVS_CAMERA_CONTROL_LIB}
)

add_executable(01-mini_visualization
    main.cpp
)

target_link_libraries(01-mini_visualization
    PRIVATE
        luxitech_mvs
)

set_target_properties(01-mini_visualization PROPERTIES
    BUILD_RPATH "${MVS_SDK_PATH}/lib/64"
)
