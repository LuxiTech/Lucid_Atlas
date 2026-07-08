#!/usr/bin/python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share_dir = get_package_share_directory("fast_livo")
    rviz_config_file = os.path.join(package_share_dir, "rviz_cfg", "cu013_camera_preview.rviz")

    camera_ws = LaunchConfiguration("camera_ws")
    sdk_runtime = LaunchConfiguration("sdk_runtime")
    sdk_env_script = LaunchConfiguration("sdk_env_script")
    sdk_version = LaunchConfiguration("sdk_version")
    use_rviz = LaunchConfiguration("use_rviz")
    external_trigger_enabled = LaunchConfiguration("external_trigger_enabled")
    trigger_source = LaunchConfiguration("trigger_source")
    trigger_activation = LaunchConfiguration("trigger_activation")

    camera_launch_cmd = [
        "source /opt/ros/humble/setup.bash && ",
        "source ",
        camera_ws,
        "/install/setup.bash && ",
        "source ",
        sdk_env_script,
        " ",
        sdk_runtime,
        " ",
        sdk_version,
        " && ",
        "ros2 launch luxi_hikrobot_cu013_camera cu013.launch.py ",
        "rviz:=false ",
        "external_trigger_enabled:=",
        external_trigger_enabled,
        " ",
        "trigger_source:=",
        trigger_source,
        " ",
        "trigger_activation:=",
        trigger_activation,
    ]

    camera_process = ExecuteProcess(
        cmd=["bash", "-lc", camera_launch_cmd],
        name="luxi_hikrobot_cu013_camera_bringup",
        output="screen",
    )

    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        name="cu013_camera_preview_rviz",
        arguments=["-d", rviz_config_file],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "camera_ws",
            default_value="/home/nvidia/project/luxi-atlas/device/luxi-hikrobot/ROS2/MV-CU013-A0UC/ws",
            description="CU013 camera ROS2 workspace path.",
        ),
        DeclareLaunchArgument(
            "sdk_runtime",
            default_value="/home/nvidia/project/luxi-atlas/device/luxi-hikrobot/SDK/runtime/MvCamCtrlSDK",
            description="Hikrobot MVS SDK runtime path.",
        ),
        DeclareLaunchArgument(
            "sdk_env_script",
            default_value="/home/nvidia/project/luxi-atlas/device/luxi-hikrobot/SDK/extracted/MvCamCtrlSDK_Runtime-4.8.0_aarch64_20260512/set_env_path.sh",
            description="Hikrobot MVS SDK environment script.",
        ),
        DeclareLaunchArgument(
            "sdk_version",
            default_value="4.8.0",
            description="Hikrobot MVS SDK version string passed to set_env_path.sh.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Open RViz2 to preview the CU013 image topic.",
        ),
        DeclareLaunchArgument(
            "external_trigger_enabled",
            default_value="false",
            description="Use external PWM trigger input when true.",
        ),
        DeclareLaunchArgument(
            "trigger_source",
            default_value="Line0",
            description="Hikrobot trigger source, usually Line0 for external PWM input.",
        ),
        DeclareLaunchArgument(
            "trigger_activation",
            default_value="RisingEdge",
            description="Hikrobot trigger activation edge.",
        ),
        camera_process,
        rviz_node,
    ])
