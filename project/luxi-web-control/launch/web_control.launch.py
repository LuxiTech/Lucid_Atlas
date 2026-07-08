import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


LOCAL_ROS_DEB_PREFIX = "/home/nvidia/project/luxi-atlas/.local_ros_debs/root/opt/ros/humble"
CYCLONEDDS_URI = (
    "<CycloneDDS><Domain><General><Interfaces>"
    '<NetworkInterface name="enP8p1s0" priority="default" multicast="default" />'
    "</Interfaces></General></Domain></CycloneDDS>"
)


def _interface_ipv4(interface_name: str) -> str:
    try:
        output = subprocess.check_output(
            ["ip", "-4", "addr", "show", "dev", interface_name],
            text=True,
            stderr=subprocess.DEVNULL,
        )
    except Exception:
        return ""

    marker = "inet "
    start = output.find(marker)
    if start < 0:
        return ""
    ip_start = start + len(marker)
    slash = output.find("/", ip_start)
    if slash < 0:
        return ""
    return output[ip_start:slash].strip()


def _launch_nodes(context):
    pkg_share = get_package_share_directory("luxi_web_control")
    web_root = os.path.join(pkg_share, "web")
    config_file = os.path.join(pkg_share, "config", "web_control.yaml")

    wifi_interface = LaunchConfiguration("wifi_interface").perform(context)
    requested_bind_address = LaunchConfiguration("bind_address").perform(context).strip()
    bind_address = requested_bind_address or "0.0.0.0"
    advertise_address = _interface_ipv4(wifi_interface)

    http_port = LaunchConfiguration("http_port")
    api_port = LaunchConfiguration("api_port")
    map_metadata_path = LaunchConfiguration("map_metadata_path")

    return [
        Node(
            package="luxi_web_control",
            executable="network_status_node",
            name="network_status_node",
            output="screen",
            parameters=[
                config_file,
                {
                    "wifi_interface": wifi_interface,
                    "bind_address": bind_address,
                    "advertise_address": advertise_address,
                    "http_port": http_port,
                    "api_port": api_port,
                },
            ],
        ),
        Node(
            package="luxi_web_control",
            executable="command_guard_node",
            name="command_guard_node",
            output="screen",
            parameters=[config_file],
        ),
        Node(
            package="luxi_web_control",
            executable="demo_state_node",
            name="demo_state_node",
            output="screen",
            parameters=[config_file],
        ),
        Node(
            package="luxi_web_control",
            executable="web_api_node",
            name="web_api_node",
            output="screen",
            parameters=[
                config_file,
                {
                    "bind_address": bind_address,
                    "api_port": api_port,
                    "web_root": web_root,
                    "map_metadata_path": map_metadata_path,
                },
            ],
        ),
        ExecuteProcess(
            cmd=[
                "python3",
                "-m",
                "http.server",
                http_port,
                "--bind",
                bind_address,
                "--directory",
                web_root,
            ],
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_legacy_http")),
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"),
            SetEnvironmentVariable("CYCLONEDDS_URI", CYCLONEDDS_URI),
            SetEnvironmentVariable(
                "AMENT_PREFIX_PATH",
                [LOCAL_ROS_DEB_PREFIX, ":", EnvironmentVariable("AMENT_PREFIX_PATH", default_value="")],
            ),
            SetEnvironmentVariable(
                "CMAKE_PREFIX_PATH",
                [LOCAL_ROS_DEB_PREFIX, ":", EnvironmentVariable("CMAKE_PREFIX_PATH", default_value="")],
            ),
            SetEnvironmentVariable(
                "LD_LIBRARY_PATH",
                [
                    LOCAL_ROS_DEB_PREFIX,
                    "/lib:",
                    LOCAL_ROS_DEB_PREFIX,
                    "/lib/aarch64-linux-gnu:",
                    EnvironmentVariable("LD_LIBRARY_PATH", default_value=""),
                ],
            ),
            DeclareLaunchArgument(
                "wifi_interface",
                default_value="wlP1p1s0",
                description="WiFi interface used for same-LAN phone access.",
            ),
            DeclareLaunchArgument(
                "bind_address",
                default_value="0.0.0.0",
                description=(
                    "Address to bind the C++ web/API server. 0.0.0.0 listens on all local "
                    "interfaces and is recommended for phones on changing WiFi networks."
                ),
            ),
            DeclareLaunchArgument(
                "http_port",
                default_value="8080",
                description="Legacy Python static HTTP port. Disabled unless start_legacy_http=true.",
            ),
            DeclareLaunchArgument(
                "api_port",
                default_value="8082",
                description="Primary C++ web/API port for local-LAN control, maps, camera, and static UI.",
            ),
            DeclareLaunchArgument(
                "map_metadata_path",
                default_value="",
                description="Current map metadata JSON used to align 3D browser clouds to the loaded floor plane.",
            ),
            DeclareLaunchArgument(
                "start_legacy_http",
                default_value="true",
                description="Start the old Python static web server on http_port for compatibility.",
            ),
            OpaqueFunction(function=_launch_nodes),
        ]
    )
