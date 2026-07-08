#!/usr/bin/python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def without_mvcam_sdk_paths():
    paths = os.environ.get("LD_LIBRARY_PATH", "").split(":")
    keep = [
        path for path in paths
        if path
        and "MvCamCtrlSDK" not in path
        and "/opt/MVS/lib" not in path
    ]
    return ":".join(keep)


def generate_launch_description():
    package_share_dir = get_package_share_directory("fast_livo")
    config_dir = os.path.join(package_share_dir, "config")
    rviz_config_file = os.path.join(package_share_dir, "rviz_cfg", "fast_livo2_lio_only.rviz")

    livo_config_file = os.path.join(config_dir, "luxi", "luxi_cu013_mid360_lio_only.yaml")
    camera_config_file = os.path.join(config_dir, "luxi", "camera_cu013.yaml")

    use_rviz = LaunchConfiguration("use_rviz")
    livo_params_file = LaunchConfiguration("livo_params_file")
    camera_params_file = LaunchConfiguration("camera_params_file")

    return LaunchDescription([
        SetEnvironmentVariable("LD_PRELOAD", "/lib/aarch64-linux-gnu/libusb-1.0.so.0"),
        SetEnvironmentVariable("LD_LIBRARY_PATH", without_mvcam_sdk_paths()),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument(
            "livo_params_file",
            default_value=livo_config_file,
            description="FAST-LIVO2 LIO-only parameters for Luxi CU013 + MID360.",
        ),
        DeclareLaunchArgument(
            "camera_params_file",
            default_value=camera_config_file,
            description="Camera intrinsics for Luxi CU013.",
        ),
        Node(
            package="fast_livo",
            executable="fastlivo_mapping",
            name="laserMapping",
            parameters=[
                livo_params_file,
                camera_params_file,
            ],
            output="screen",
        ),
        Node(
            condition=IfCondition(use_rviz),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config_file],
            output="screen",
        ),
    ])
