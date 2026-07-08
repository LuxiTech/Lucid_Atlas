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
    rviz_config_file = os.path.join(package_share_dir, "rviz_cfg", "device_cu013_mid360_preview.rviz")

    use_camera = LaunchConfiguration("use_camera")
    use_lidar = LaunchConfiguration("use_lidar")
    use_rviz = LaunchConfiguration("use_rviz")

    camera_ws = LaunchConfiguration("camera_ws")
    camera_launch = LaunchConfiguration("camera_launch")
    sdk_runtime = LaunchConfiguration("sdk_runtime")
    sdk_env_script = LaunchConfiguration("sdk_env_script")
    sdk_version = LaunchConfiguration("sdk_version")
    external_trigger_enabled = LaunchConfiguration("external_trigger_enabled")
    trigger_source = LaunchConfiguration("trigger_source")
    trigger_activation = LaunchConfiguration("trigger_activation")

    mid360_ws = LaunchConfiguration("mid360_ws")
    mid360_config = LaunchConfiguration("mid360_config")
    mid360_xfer_format = LaunchConfiguration("mid360_xfer_format")
    mid360_frame_id = LaunchConfiguration("mid360_frame_id")
    mid360_publish_freq = LaunchConfiguration("mid360_publish_freq")

    camera_cmd = [
        "source /opt/ros/humble/setup.bash && ",
        "source ", camera_ws, "/install/setup.bash && ",
        "source ", sdk_env_script, " ", sdk_runtime, " ", sdk_version, " && ",
        "ros2 launch luxi_hikrobot_cu013_camera ", camera_launch, " ",
        "rviz:=false ",
        "external_trigger_enabled:=", external_trigger_enabled, " ",
        "trigger_source:=", trigger_source, " ",
        "trigger_activation:=", trigger_activation,
    ]

    mid360_cmd = [
        "source /opt/ros/humble/setup.bash && ",
        "source ", mid360_ws, "/install/setup.bash && ",
        "ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args ",
        "-p xfer_format:=", mid360_xfer_format, " ",
        "-p multi_topic:=0 ",
        "-p data_src:=0 ",
        "-p publish_freq:=", mid360_publish_freq, " ",
        "-p output_data_type:=0 ",
        "-p frame_id:=", mid360_frame_id, " ",
        "-p user_config_path:=", mid360_config,
    ]

    camera_process = ExecuteProcess(
        condition=IfCondition(use_camera),
        cmd=["bash", "-lc", camera_cmd],
        name="cu013_camera_device",
        output="screen",
    )

    mid360_process = ExecuteProcess(
        condition=IfCondition(use_lidar),
        cmd=["bash", "-lc", mid360_cmd],
        name="mid360_livox_device",
        output="screen",
    )

    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        name="device_cu013_mid360_preview_rviz",
        arguments=["-d", rviz_config_file],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_camera", default_value="true"),
        DeclareLaunchArgument("use_lidar", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument(
            "camera_ws",
            default_value="/home/nvidia/project/luxi-atlas/device/luxi-hikrobot/ROS2/MV-CU013-A0UC/ws",
        ),
        DeclareLaunchArgument("camera_launch", default_value="cu013.launch.py"),
        DeclareLaunchArgument(
            "sdk_runtime",
            default_value="/home/nvidia/project/luxi-atlas/device/luxi-hikrobot/SDK/runtime/MvCamCtrlSDK",
        ),
        DeclareLaunchArgument(
            "sdk_env_script",
            default_value="/home/nvidia/project/luxi-atlas/device/luxi-hikrobot/SDK/extracted/MvCamCtrlSDK_Runtime-4.8.0_aarch64_20260512/set_env_path.sh",
        ),
        DeclareLaunchArgument("sdk_version", default_value="4.8.0"),
        DeclareLaunchArgument("external_trigger_enabled", default_value="true"),
        DeclareLaunchArgument("trigger_source", default_value="Line0"),
        DeclareLaunchArgument("trigger_activation", default_value="RisingEdge"),
        DeclareLaunchArgument(
            "mid360_ws",
            default_value="/home/nvidia/project/luxi-atlas/device/mid360",
        ),
        DeclareLaunchArgument(
            "mid360_config",
            default_value="/home/nvidia/project/luxi-atlas/device/mid360/src/livox_ros_driver2/config/MID360_config.json",
        ),
        DeclareLaunchArgument(
            "mid360_xfer_format",
            default_value="0",
            description="0 publishes PointCloud2 for RViz preview; 1 publishes Livox CustomMsg for FAST-LIVO2.",
        ),
        DeclareLaunchArgument("mid360_frame_id", default_value="livox_frame"),
        DeclareLaunchArgument("mid360_publish_freq", default_value="10.0"),
        camera_process,
        mid360_process,
        rviz_node,
    ])
