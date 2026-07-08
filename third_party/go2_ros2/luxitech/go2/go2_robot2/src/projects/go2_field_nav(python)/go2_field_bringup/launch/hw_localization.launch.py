import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rviz = LaunchConfiguration("rviz")
    enable_cmd_vel_bridge = LaunchConfiguration("enable_cmd_vel_bridge")
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    auto_initial_pose = LaunchConfiguration("auto_initial_pose")
    initial_pose_x = LaunchConfiguration("initial_pose_x")
    initial_pose_y = LaunchConfiguration("initial_pose_y")
    initial_pose_yaw = LaunchConfiguration("initial_pose_yaw")
    use_global_localization = LaunchConfiguration("use_global_localization")
    enable_scan_bridge = LaunchConfiguration("enable_scan_bridge")
    enable_pointcloud_relay = LaunchConfiguration("enable_pointcloud_relay")
    enable_joint_state_bridge = LaunchConfiguration("enable_joint_state_bridge")
    enable_front_video_bridge = LaunchConfiguration("enable_front_video_bridge")
    sport_state_topic = LaunchConfiguration("sport_state_topic")
    lowstate_topic = LaunchConfiguration("lowstate_topic")
    enable_fake_odom = LaunchConfiguration("enable_fake_odom")
    enable_fake_map_to_odom = LaunchConfiguration("enable_fake_map_to_odom")

    bringup_share = get_package_share_directory("go2_field_bringup")
    config_share = get_package_share_directory("go2_field_config")
    nav2_bringup_share = get_package_share_directory("nav2_bringup")

    default_map = os.path.join(config_share, "maps", "lab_map.yaml")
    default_params = os.path.join(config_share, "config", "nav2", "nav2_hw_params.yaml")
    default_rviz = os.path.join(config_share, "rviz", "hw_nav2_view.rviz")

    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "hw_sensors.launch.py")
        ),
        launch_arguments={
            "rviz": "false",
            "use_sim_time": use_sim_time,
            "enable_cmd_vel_bridge": enable_cmd_vel_bridge,
            "enable_scan_bridge": enable_scan_bridge,
            "enable_pointcloud_relay": enable_pointcloud_relay,
            "enable_joint_state_bridge": enable_joint_state_bridge,
            "enable_front_video_bridge": enable_front_video_bridge,
            "sport_state_topic": sport_state_topic,
            "lowstate_topic": lowstate_topic,
            "enable_fake_odom": enable_fake_odom,
        }.items(),
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, "launch", "localization_launch.py")
        ),
        launch_arguments={
            "map": map_yaml,
            "params_file": params_file,
            "use_sim_time": use_sim_time,
            "autostart": "true",
            "use_composition": "False",
            "use_respawn": "False",
        }.items(),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_hw_localization",
        arguments=["-d", default_rviz],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(rviz),
        output="screen",
    )

    initial_pose_node = Node(
        package="go2_field_bringup",
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
        package="go2_field_bringup",
        executable="global_localization_trigger.py",
        name="global_localization_trigger",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_global_localization),
        output="screen",
    )

    fake_map_to_odom_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="fake_map_to_odom_tf",
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            "0.0",
            "--roll",
            "0.0",
            "--pitch",
            "0.0",
            "--yaw",
            "0.0",
            "--frame-id",
            "map",
            "--child-frame-id",
            "odom",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(enable_fake_map_to_odom),
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("enable_cmd_vel_bridge", default_value="false"),
            DeclareLaunchArgument("enable_scan_bridge", default_value="false"),
            DeclareLaunchArgument("enable_pointcloud_relay", default_value="true"),
            DeclareLaunchArgument("enable_joint_state_bridge", default_value="true"),
            DeclareLaunchArgument("enable_front_video_bridge", default_value="false"),
            DeclareLaunchArgument("sport_state_topic", default_value="/lf/sportmodestate"),
            DeclareLaunchArgument("lowstate_topic", default_value="/lowstate"),
            DeclareLaunchArgument("enable_fake_odom", default_value="false"),
            DeclareLaunchArgument("enable_fake_map_to_odom", default_value="false"),
            DeclareLaunchArgument("map", default_value=default_map),
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("auto_initial_pose", default_value="false"),
            DeclareLaunchArgument("initial_pose_x", default_value="0.0"),
            DeclareLaunchArgument("initial_pose_y", default_value="0.0"),
            DeclareLaunchArgument("initial_pose_yaw", default_value="0.0"),
            DeclareLaunchArgument("use_global_localization", default_value="false"),
            sensors_launch,
            localization_launch,
            initial_pose_node,
            global_localization_node,
            fake_map_to_odom_tf_node,
            rviz_node,
        ]
    )
