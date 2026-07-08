import glob
import json
import os
import re

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


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
    selected_json = ""
    for candidate in json_candidates:
        try:
            with open(candidate, "r", encoding="utf-8") as metadata_file:
                metadata = json.load(metadata_file)
            if os.path.abspath(metadata.get("input", "")) == os.path.abspath(pcd_path):
                selected_json = candidate
                break
        except Exception:
            continue
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
    package_share = FindPackageShare("luxi_localization")
    livox_share = FindPackageShare("livox_ros_driver2")
    fast_lio_share = FindPackageShare("fast_lio")

    default_config = PathJoinSubstitution(
        [package_share, "config", "localization.yaml"]
    )
    default_rviz_config = PathJoinSubstitution(
        [package_share, "rviz", "luxi_localization.rviz"]
    )
    default_livox_launch = PathJoinSubstitution(
        [livox_share, "launch_ROS2", "msg_MID360_launch.py"]
    )
    default_fast_lio_launch = PathJoinSubstitution(
        [fast_lio_share, "launch", "mapping.launch.py"]
    )
    default_pcd_map, default_grid_map_yaml, default_grid_map_json = _default_map_paths()

    return LaunchDescription(
        [
            SetEnvironmentVariable(
                "LD_PRELOAD",
                "/usr/lib/aarch64-linux-gnu/libusb-1.0.so.0",
            ),
            DeclareLaunchArgument("start_lidar", default_value="true"),
            DeclareLaunchArgument("start_fast_lio", default_value="true"),
            DeclareLaunchArgument("start_localization", default_value="true"),
            DeclareLaunchArgument("publish_grid_map", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("livox_launch", default_value=default_livox_launch),
            DeclareLaunchArgument("fast_lio_launch", default_value=default_fast_lio_launch),
            DeclareLaunchArgument(
                "fast_lio_config_path",
                default_value="/home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/config",
            ),
            DeclareLaunchArgument("fast_lio_config_file", default_value="mid360.yaml"),
            DeclareLaunchArgument("config_file", default_value=default_config),
            DeclareLaunchArgument(
                "map_path",
                default_value=default_pcd_map,
            ),
            DeclareLaunchArgument("pcd_map", default_value=LaunchConfiguration("map_path")),
            DeclareLaunchArgument(
                "grid_map_yaml",
                default_value=default_grid_map_yaml,
            ),
            DeclareLaunchArgument("pgm_map", default_value=LaunchConfiguration("grid_map_yaml")),
            DeclareLaunchArgument(
                "grid_map_json",
                default_value=default_grid_map_json,
            ),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(LaunchConfiguration("livox_launch")),
                condition=IfCondition(LaunchConfiguration("start_lidar")),
            ),
            Node(
                package="fast_lio",
                executable="fastlio_mapping",
                name="fastlio_mapping",
                output="screen",
                respawn=True,
                respawn_delay=1.0,
                parameters=[
                    ParameterFile(
                        PathJoinSubstitution(
                            [
                                LaunchConfiguration("fast_lio_config_path"),
                                LaunchConfiguration("fast_lio_config_file"),
                            ]
                        ),
                        allow_substs=True,
                    ),
                    {"use_sim_time": False},
                ],
                condition=IfCondition(LaunchConfiguration("start_fast_lio")),
            ),
            Node(
                package="luxi_localization",
                executable="grid_map_publisher",
                name="luxi_grid_map_publisher",
                output="screen",
                parameters=[
                    {"map_yaml": LaunchConfiguration("pgm_map")},
                ],
                condition=IfCondition(LaunchConfiguration("publish_grid_map")),
            ),
            Node(
                package="luxi_localization",
                executable="localization_node",
                name="luxi_open3d_localization",
                output="screen",
                parameters=[
                    ParameterFile(LaunchConfiguration("config_file"), allow_substs=True),
                    {"map_path": LaunchConfiguration("pcd_map")},
                    {"map_metadata_path": LaunchConfiguration("grid_map_json")},
                ],
                condition=IfCondition(LaunchConfiguration("start_localization")),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", LaunchConfiguration("rviz_config")],
                condition=IfCondition(LaunchConfiguration("use_rviz")),
            ),
        ]
    )
