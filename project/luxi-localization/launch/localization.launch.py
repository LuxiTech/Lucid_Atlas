import glob
import json
import os
import re

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


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

    if not selected_json:
        selected_json = (
            "/home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/tools/"
            "transfrom_tools/pcd_to_pgm/output/"
            "test_map5_nav_floor_plane_autofit_h005_050_clean80/map.json"
        )
    return pcd_path, selected_json


def generate_launch_description():
    default_config = PathJoinSubstitution(
        [FindPackageShare("luxi_localization"), "config", "localization.yaml"]
    )
    default_pcd_map, default_map_metadata = _default_map_paths()

    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=default_config),
            DeclareLaunchArgument(
                "map_path",
                default_value=default_pcd_map,
            ),
            DeclareLaunchArgument("pcd_map", default_value=LaunchConfiguration("map_path")),
            DeclareLaunchArgument(
                "map_metadata_path",
                default_value=default_map_metadata,
            ),
            Node(
                package="luxi_localization",
                executable="localization_node",
                name="luxi_open3d_localization",
                output="screen",
                parameters=[
                    LaunchConfiguration("config_file"),
                    {"map_path": LaunchConfiguration("pcd_map")},
                    {"map_metadata_path": LaunchConfiguration("map_metadata_path")},
                ],
            ),
        ]
    )
