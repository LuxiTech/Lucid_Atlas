import glob
import json
import os
import re

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


LOCAL_ROS_DEB_PREFIX = "/home/nvidia/project/luxi-atlas/.local_ros_debs/root/opt/ros/humble"
CYCLONEDDS_URI = (
    "<CycloneDDS><Domain><General><Interfaces>"
    '<NetworkInterface name="enP8p1s0" priority="default" multicast="default" />'
    "</Interfaces></General></Domain></CycloneDDS>"
)
MAPS_DIR = "/home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/maps"
GRID_OUTPUT_DIR = (
    "/home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/tools/"
    "transfrom_tools/pcd_to_pgm/output"
)


def _map_sort_key(path):
    stem = os.path.splitext(os.path.basename(path))[0]
    match = re.search(r"(\d+)$", stem)
    number = int(match.group(1)) if match else -1
    try:
        mtime = os.path.getmtime(path)
    except OSError:
        mtime = 0.0
    return number, mtime


def _default_map_paths():
    pcd_candidates = glob.glob(os.path.join(MAPS_DIR, "test_map*.pcd"))
    pcd_path = (
        max(pcd_candidates, key=_map_sort_key)
        if pcd_candidates
        else os.path.join(MAPS_DIR, "test_map5.pcd")
    )
    stem = os.path.splitext(os.path.basename(pcd_path))[0]
    json_candidates = glob.glob(os.path.join(GRID_OUTPUT_DIR, f"{stem}*", "map.json"))
    matching_json = []
    for candidate in json_candidates:
        try:
            with open(candidate, "r", encoding="utf-8") as metadata_file:
                metadata = json.load(metadata_file)
            input_path = metadata.get("input", "")
            if os.path.abspath(input_path) == os.path.abspath(pcd_path):
                matching_json.append(candidate)
        except Exception:
            continue
    selected_json = max(matching_json, key=lambda path: os.path.getmtime(path)) if matching_json else ""
    if not selected_json and json_candidates:
        selected_json = max(json_candidates, key=lambda path: os.path.getmtime(path))

    selected_yaml = ""
    if selected_json:
        try:
            with open(selected_json, "r", encoding="utf-8") as metadata_file:
                metadata = json.load(metadata_file)
            selected_yaml = metadata.get("output_yaml", "")
        except Exception:
            selected_yaml = ""
        if not selected_yaml:
            selected_yaml = os.path.join(os.path.dirname(selected_json), "map.yaml")

    if not selected_yaml:
        selected_yaml = (
            "/home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/tools/"
            "transfrom_tools/pcd_to_pgm/output/"
            "test_map5_nav_floor_plane_autofit_h005_050_clean80/map.yaml"
        )
    if not selected_json:
        selected_json = (
            "/home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/tools/"
            "transfrom_tools/pcd_to_pgm/output/"
            "test_map5_nav_floor_plane_autofit_h005_050_clean80/map.json"
        )
    return pcd_path, selected_yaml, selected_json


def generate_launch_description():
    web_share = FindPackageShare("luxi_web_control")
    localization_share = FindPackageShare("luxi_localization")
    navigation_share = FindPackageShare("luxi_navigation")

    camera_setup = LaunchConfiguration("camera_setup")
    camera_launch = LaunchConfiguration("camera_launch")
    go2_bridge_script = LaunchConfiguration("go2_bridge_script")
    default_pcd_map, default_grid_map_yaml, default_grid_map_json = _default_map_paths()

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
            DeclareLaunchArgument("start_camera", default_value="true"),
            DeclareLaunchArgument("start_localization", default_value="true"),
            DeclareLaunchArgument("start_navigation", default_value="true"),
            DeclareLaunchArgument("start_go2_bridge", default_value="false"),
            DeclareLaunchArgument("start_web", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("wifi_interface", default_value="wlP1p1s0"),
            DeclareLaunchArgument("bind_address", default_value=""),
            DeclareLaunchArgument("http_port", default_value="8080"),
            DeclareLaunchArgument("api_port", default_value="8082"),
            DeclareLaunchArgument("start_legacy_http", default_value="true"),
            DeclareLaunchArgument("map_path", default_value=default_pcd_map),
            DeclareLaunchArgument("pcd_map", default_value=LaunchConfiguration("map_path")),
            DeclareLaunchArgument("grid_map_yaml", default_value=default_grid_map_yaml),
            DeclareLaunchArgument("pgm_map", default_value=LaunchConfiguration("grid_map_yaml")),
            DeclareLaunchArgument("grid_map_json", default_value=default_grid_map_json),
            DeclareLaunchArgument(
                "camera_setup",
                default_value=(
                    "/home/nvidia/project/luxi-atlas/device/luxi-hikrobot/"
                    "ROS2/MV-CU013-A0UC/ws/install/setup.bash"
                ),
            ),
            DeclareLaunchArgument(
                "camera_launch",
                default_value="luxi_hikrobot_cu013_camera cu013.launch.py",
            ),
            DeclareLaunchArgument(
                "go2_bridge_script",
                default_value="/home/nvidia/project/luxi-atlas/scripts/start_go2_cmd_vel_bridge.sh",
            ),
            ExecuteProcess(
                cmd=[
                    "bash",
                    "-lc",
                    [
                        "source ",
                        camera_setup,
                        " && exec ros2 launch ",
                        camera_launch,
                        " rviz:=false",
                    ],
                ],
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_camera")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [localization_share, "launch", "bringup_localization.launch.py"]
                    )
                ),
                launch_arguments={
                    "use_rviz": LaunchConfiguration("use_rviz"),
                    "start_localization": LaunchConfiguration("start_localization"),
                    "pcd_map": LaunchConfiguration("pcd_map"),
                    "pgm_map": LaunchConfiguration("pgm_map"),
                    "grid_map_json": LaunchConfiguration("grid_map_json"),
                }.items(),
                condition=IfCondition(LaunchConfiguration("start_localization")),
            ),
            TimerAction(
                period=3.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution(
                                [navigation_share, "launch", "navigation.launch.py"]
                            )
                        ),
                    )
                ],
                condition=IfCondition(LaunchConfiguration("start_navigation")),
            ),
            TimerAction(
                period=5.0,
                actions=[
                    ExecuteProcess(
                        cmd=["bash", "-lc", ["exec bash ", go2_bridge_script]],
                        output="screen",
                    )
                ],
                condition=IfCondition(LaunchConfiguration("start_go2_bridge")),
            ),
            TimerAction(
                period=4.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution(
                                [web_share, "launch", "web_control.launch.py"]
                            )
                        ),
                        launch_arguments={
                            "wifi_interface": LaunchConfiguration("wifi_interface"),
                            "bind_address": LaunchConfiguration("bind_address"),
                            "http_port": LaunchConfiguration("http_port"),
                            "api_port": LaunchConfiguration("api_port"),
                            "start_legacy_http": LaunchConfiguration("start_legacy_http"),
                            "map_metadata_path": LaunchConfiguration("grid_map_json"),
                        }.items(),
                    )
                ],
                condition=IfCondition(LaunchConfiguration("start_web")),
            ),
        ]
    )
