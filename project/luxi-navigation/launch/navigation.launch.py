from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("luxi_navigation")
    default_config = PathJoinSubstitution([package_share, "config", "navigation.yaml"])

    return LaunchDescription(
        [
            DeclareLaunchArgument("navigation_config_file", default_value=default_config),
            Node(
                package="luxi_navigation",
                executable="luxi_navigation_node",
                name="luxi_navigation_node",
                output="screen",
                parameters=[LaunchConfiguration("navigation_config_file")],
            ),
        ]
    )
