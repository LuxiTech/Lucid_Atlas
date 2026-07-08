import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
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
    paused = LaunchConfiguration("paused")
    auto_unpause = LaunchConfiguration("auto_unpause")
    unpause_delay = LaunchConfiguration("unpause_delay")
    robot_name = LaunchConfiguration("robot_name")
    map_yaml = LaunchConfiguration("map")
    nav2_params_file = LaunchConfiguration("params_file")
    nav2_rviz = LaunchConfiguration("nav2_rviz")
    nav2_rviz_config = LaunchConfiguration("nav2_rviz_config")
    auto_initial_pose = LaunchConfiguration("auto_initial_pose")
    initial_pose_x = LaunchConfiguration("initial_pose_x")
    initial_pose_y = LaunchConfiguration("initial_pose_y")
    initial_pose_yaw = LaunchConfiguration("initial_pose_yaw")
    use_global_localization = LaunchConfiguration("use_global_localization")

    mapping_lab_bringup_share = get_package_share_directory("mapping_lab_bringup")
    mapping_lab_config_share = get_package_share_directory("mapping_lab_config")
    nav2_bringup_share = get_package_share_directory("nav2_bringup")

    default_world_path = os.path.join(
        mapping_lab_config_share, "worlds", "turtlebot3_house_classic.world"
    )
    default_map_path = os.path.join(mapping_lab_config_share, "maps", "lab_map.yaml")
    default_nav2_params = os.path.join(
        mapping_lab_config_share, "config", "nav2", "nav2_sim_params.yaml"
    )
    default_rviz_config = os.path.join(
        mapping_lab_config_share, "rviz", "nav2_view.rviz"
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
            "paused": paused,
            "auto_unpause": auto_unpause,
            "unpause_delay": unpause_delay,
            "robot_name": robot_name,
            "enable_rviz": "false",
        }.items(),
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "slam": "False",
            "map": map_yaml,
            "params_file": nav2_params_file,
            "use_sim_time": use_sim_time,
            "autostart": "true",
            "use_composition": "False",
            "use_respawn": "False",
        }.items(),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_nav2",
        arguments=["-d", nav2_rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(nav2_rviz),
        output="screen",
    )

    initial_pose_node = Node(
        package="mapping_lab_bringup",
        executable="initial_pose_publisher.py",
        name="initial_pose_publisher",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"x": initial_pose_x},
            {"y": initial_pose_y},
            {"yaw": initial_pose_yaw},
        ],
        condition=IfCondition(auto_initial_pose),
        output="screen",
    )

    global_localization_node = Node(
        package="mapping_lab_bringup",
        executable="global_localization_trigger.py",
        name="global_localization_trigger",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_global_localization),
        output="screen",
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
            DeclareLaunchArgument("paused", default_value="false"),
            DeclareLaunchArgument("auto_unpause", default_value="false"),
            DeclareLaunchArgument("unpause_delay", default_value="2.0"),
            DeclareLaunchArgument("map", default_value=default_map_path),
            DeclareLaunchArgument("params_file", default_value=default_nav2_params),
            DeclareLaunchArgument("nav2_rviz", default_value="true"),
            DeclareLaunchArgument("nav2_rviz_config", default_value=default_rviz_config),
            DeclareLaunchArgument("auto_initial_pose", default_value="true"),
            DeclareLaunchArgument("initial_pose_x", default_value="1.5"),
            DeclareLaunchArgument("initial_pose_y", default_value="-2.0"),
            DeclareLaunchArgument("initial_pose_yaw", default_value="0.0"),
            DeclareLaunchArgument("use_global_localization", default_value="false"),
            sim_launch,
            nav2_launch,
            rviz_node,
            initial_pose_node,
            global_localization_node,
        ]
    )
