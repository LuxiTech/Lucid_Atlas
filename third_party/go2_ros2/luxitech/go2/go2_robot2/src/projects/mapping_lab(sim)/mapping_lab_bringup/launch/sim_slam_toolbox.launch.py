import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    world = LaunchConfiguration("world")
    world_init_x = LaunchConfiguration("world_init_x")
    world_init_y = LaunchConfiguration("world_init_y")
    world_init_z = LaunchConfiguration("world_init_z")
    world_init_heading = LaunchConfiguration("world_init_heading")
    robot_name = LaunchConfiguration("robot_name")

    mapping_lab_config_share = get_package_share_directory("mapping_lab_config")
    mapping_lab_bringup_share = get_package_share_directory("mapping_lab_bringup")

    slam_params_file = os.path.join(
        mapping_lab_config_share,
        "config",
        "slam_toolbox",
        "mapper_params_online_async.yaml",
    )
    slam_rviz_config = os.path.join(
        mapping_lab_config_share, "rviz", "slam_toolbox_view.rviz"
    )

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mapping_lab_bringup_share, "launch", "sim_lidar_view.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "gui": gui,
            "world": world,
            "world_init_x": world_init_x,
            "world_init_y": world_init_y,
            "world_init_z": world_init_z,
            "world_init_heading": world_init_heading,
            "enable_rviz": "false",
            "robot_name": robot_name,
        }.items(),
    )

    slam_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_params_file, {"use_sim_time": use_sim_time}],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_slam",
        arguments=["-d", slam_rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    default_world_path = os.path.join(
        mapping_lab_config_share, "worlds", "turtlebot3_house_classic.world"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("robot_name", default_value="go2"),
            DeclareLaunchArgument("world", default_value=default_world_path),
            DeclareLaunchArgument("world_init_x", default_value="1.5"),
            DeclareLaunchArgument("world_init_y", default_value="-2.0"),
            DeclareLaunchArgument("world_init_z", default_value="0.35"),
            DeclareLaunchArgument("world_init_heading", default_value="0.0"),
            sim_launch,
            slam_node,
            rviz_node,
        ]
    )
