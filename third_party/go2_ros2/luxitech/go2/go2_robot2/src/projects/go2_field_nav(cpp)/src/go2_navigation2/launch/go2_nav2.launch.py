import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav2_pkg = get_package_share_directory("go2_navigation2")
    nav2_bringup_pkg = get_package_share_directory("nav2_bringup")
    go2_core_pkg = get_package_share_directory("go2_core")
    go2_driver_pkg = get_package_share_directory("go2_driver")
    go2_perception_pkg = get_package_share_directory("go2_perception")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    map_yaml_path = LaunchConfiguration("map")
    nav2_param_path = LaunchConfiguration("params_file")
    rviz_config = LaunchConfiguration("rviz_config")

    go2_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(go2_driver_pkg, "launch", "driver.launch.py")
        ),
        launch_arguments={"use_rviz": "false"}.items(),
    )

    go2_robot_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(go2_core_pkg, "launch", "go2_robot_localization.launch.py")
        )
    )

    go2_pointcloud_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(go2_perception_pkg, "launch", "go2_pointcloud.launch.py")
        )
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_pkg, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "params_file": nav2_param_path,
            "use_sim_time": use_sim_time,
            "map": map_yaml_path,
        }.items(),
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_nav2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_rviz),
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument(
            "map",
            default_value=os.path.join(nav2_pkg, "maps", "lab_map.yaml"),
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(nav2_pkg, "config", "nav2_params.yaml"),
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=os.path.join(nav2_bringup_pkg, "rviz", "nav2_default_view.rviz"),
        ),
        go2_driver_launch,
        go2_robot_localization,
        go2_pointcloud_launch,
        nav2_launch,
        rviz2,
    ])
