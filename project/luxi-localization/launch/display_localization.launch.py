from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("luxi_localization")
    default_config = PathJoinSubstitution(
        [package_share, "config", "localization.yaml"]
    )
    default_rviz_config = PathJoinSubstitution(
        [package_share, "rviz", "luxi_localization.rviz"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=default_config),
            DeclareLaunchArgument(
                "map_path",
                default_value="/home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/PCD/fast_lio_map_20260622_162740.pcd",
            ),
            DeclareLaunchArgument("pcd_map", default_value=LaunchConfiguration("map_path")),
            DeclareLaunchArgument(
                "grid_map_yaml",
                default_value="/home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/tools/transfrom_tools/pcd_to_pgm/output/fast_lio_map_20260622_162740_nav_floor_plane_clean20/map.yaml",
            ),
            DeclareLaunchArgument("pgm_map", default_value=LaunchConfiguration("grid_map_yaml")),
            DeclareLaunchArgument("publish_grid_map", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
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
                ],
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
