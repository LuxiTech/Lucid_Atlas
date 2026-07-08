import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    rviz = LaunchConfiguration("rviz")
    rviz_mode = LaunchConfiguration("rviz_mode")
    rviz_config = LaunchConfiguration("rviz_config")
    enable_cmd_vel_bridge = LaunchConfiguration("enable_cmd_vel_bridge")
    enable_keyboard_teleop = LaunchConfiguration("enable_keyboard_teleop")
    enable_watchdog = LaunchConfiguration("enable_watchdog")
    watchdog_topic = LaunchConfiguration("watchdog_topic")
    watchdog_timeout = LaunchConfiguration("watchdog_timeout")
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_fake_odom = LaunchConfiguration("enable_fake_odom")
    fake_odom_x = LaunchConfiguration("fake_odom_x")
    fake_odom_y = LaunchConfiguration("fake_odom_y")
    fake_odom_yaw = LaunchConfiguration("fake_odom_yaw")
    sport_state_topic = LaunchConfiguration("sport_state_topic")
    enable_pointcloud_relay = LaunchConfiguration("enable_pointcloud_relay")
    enable_joint_state_bridge = LaunchConfiguration("enable_joint_state_bridge")
    enable_front_video_bridge = LaunchConfiguration("enable_front_video_bridge")
    lidar_cloud_topic = LaunchConfiguration("lidar_cloud_topic")
    pointcloud_topic = LaunchConfiguration("pointcloud_topic")
    pointcloud_frame_id = LaunchConfiguration("pointcloud_frame_id")
    scan_target_frame = LaunchConfiguration("scan_target_frame")
    scan_min_height = LaunchConfiguration("scan_min_height")
    scan_max_height = LaunchConfiguration("scan_max_height")
    scan_range_min = LaunchConfiguration("scan_range_min")
    scan_range_max = LaunchConfiguration("scan_range_max")
    enable_scan_boundary_filter = LaunchConfiguration("enable_scan_boundary_filter")
    scan_input_topic = LaunchConfiguration("scan_input_topic")
    scan_output_topic = LaunchConfiguration("scan_output_topic")
    enable_scan_tuning_gui = LaunchConfiguration("enable_scan_tuning_gui")
    scan_tuning_file = LaunchConfiguration("scan_tuning_file")
    lowstate_topic = LaunchConfiguration("lowstate_topic")
    front_video_topic = LaunchConfiguration("front_video_topic")
    front_image_topic = LaunchConfiguration("front_image_topic")
    keyboard_cmd_topic = LaunchConfiguration("keyboard_cmd_topic")
    keyboard_linear_speed = LaunchConfiguration("keyboard_linear_speed")
    keyboard_lateral_speed = LaunchConfiguration("keyboard_lateral_speed")
    keyboard_angular_speed = LaunchConfiguration("keyboard_angular_speed")
    keyboard_key_timeout = LaunchConfiguration("keyboard_key_timeout")
    keyboard_publish_hz = LaunchConfiguration("keyboard_publish_hz")

    go2_description_share = get_package_share_directory("go2_description")
    go2_field_config_share = get_package_share_directory("go2_field_config")
    enable_scan_bridge = LaunchConfiguration("enable_scan_bridge")

    launch_dir = os.path.join(go2_description_share, "launch")
    description_launch = os.path.join(launch_dir, "description.launch.py")
    robot_launch = os.path.join(launch_dir, "robot.launch.py")
    description_entry = (
        description_launch if os.path.exists(description_launch) else robot_launch
    )

    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(description_entry),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    cmd_bridge_condition = IfCondition(
        PythonExpression(
            [
                "'",
                enable_cmd_vel_bridge,
                "' == 'true' or '",
                enable_keyboard_teleop,
                "' == 'true'",
            ]
        )
    )

    official_cmd_vel_bridge_node = Node(
        package="go2_field_bringup",
        executable="official_cmd_vel_bridge.py",
        name="official_cmd_vel_bridge",
        condition=cmd_bridge_condition,
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    keyboard_teleop_node = Node(
        package="go2_field_bringup",
        executable="keyboard_teleop.py",
        name="keyboard_teleop",
        condition=IfCondition(enable_keyboard_teleop),
        parameters=[
            {"use_sim_time": use_sim_time},
            {"cmd_vel_topic": keyboard_cmd_topic},
            {"linear_speed": keyboard_linear_speed},
            {"lateral_speed": keyboard_lateral_speed},
            {"angular_speed": keyboard_angular_speed},
            {"key_timeout_sec": keyboard_key_timeout},
            {"publish_hz": keyboard_publish_hz},
        ],
        emulate_tty=True,
        output="screen",
    )

    sport_state_bridge_node = Node(
        package="go2_field_bringup",
        executable="sport_state_bridge.py",
        name="sport_state_bridge",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"sport_state_topic": sport_state_topic},
        ],
        output="screen",
    )

    lowstate_joint_state_bridge_node = Node(
        package="go2_field_bringup",
        executable="lowstate_joint_state_bridge.py",
        name="lowstate_joint_state_bridge",
        condition=IfCondition(enable_joint_state_bridge),
        parameters=[
            {"use_sim_time": use_sim_time},
            {"lowstate_topic": lowstate_topic},
        ],
        output="screen",
    )

    pointcloud_relay_node = Node(
        package="go2_field_bringup",
        executable="pointcloud_relay.py",
        name="pointcloud_relay",
        condition=IfCondition(enable_pointcloud_relay),
        parameters=[
            {"use_sim_time": use_sim_time},
            {"input_topic": lidar_cloud_topic},
            {"output_topic": pointcloud_topic},
            {"force_frame_id": pointcloud_frame_id},
        ],
        output="screen",
    )

    front_video_bridge_node = Node(
        package="go2_field_bringup",
        executable="front_video_bridge.py",
        name="front_video_bridge",
        condition=IfCondition(enable_front_video_bridge),
        parameters=[
            {"use_sim_time": use_sim_time},
            {"input_topic": front_video_topic},
            {"output_topic": front_image_topic},
        ],
        output="screen",
    )

    pointcloud_to_laserscan_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        namespace="",
        output="screen",
        remappings=[("/cloud_in", pointcloud_topic), ("/scan", "/scan")],
        parameters=[
            {
                "target_frame": scan_target_frame,
                "transform_tolerance": 0.01,
                "min_height": scan_min_height,
                "max_height": scan_max_height,
                "range_min": scan_range_min,
                "range_max": scan_range_max,
            }
        ],
        condition=IfCondition(enable_scan_bridge),
    )

    scan_boundary_filter_node = Node(
        package="go2_field_bringup",
        executable="scan_boundary_filter.py",
        name="scan_boundary_filter",
        output="screen",
        parameters=[
            scan_tuning_file,
            {
                "input_topic": scan_input_topic,
                "output_topic": scan_output_topic,
            }
        ],
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    enable_scan_bridge,
                    "' == 'true' and '",
                    enable_scan_boundary_filter,
                    "' == 'true'",
                ]
            )
        ),
    )

    scan_tuning_gui_node = Node(
        package="go2_field_bringup",
        executable="scan_tuning_gui.py",
        name="scan_tuning_gui",
        output="screen",
        condition=IfCondition(enable_scan_tuning_gui),
        additional_env={"GO2_SCAN_TUNING_FILE": scan_tuning_file},
    )

    # RViz mode:
    # - official: use go2_rviz package launch
    # - scan2d:   use local config focused on 2D LaserScan visualization
    rviz_launch = None
    rviz_node = None
    local_rviz_condition = None
    try:
        go2_rviz_share = get_package_share_directory("go2_rviz")
        rviz_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(go2_rviz_share, "launch", "rviz.launch.py")
            ),
            condition=IfCondition(
                PythonExpression(
                    ["'", rviz, "' == 'true' and '", rviz_mode, "' == 'official'"]
                )
            ),
        )
        local_rviz_condition = IfCondition(
            PythonExpression(
                ["'", rviz, "' == 'true' and '", rviz_mode, "' != 'official'"]
            )
        )
    except PackageNotFoundError:
        local_rviz_condition = IfCondition(rviz)

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_hw_sensors",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=local_rviz_condition,
        output="screen",
    )

    watchdog_node = Node(
        package="go2_field_bringup",
        executable="cmd_vel_watchdog.py",
        name="cmd_vel_watchdog",
        output="screen",
        condition=IfCondition(enable_watchdog),
        parameters=[
            {"use_sim_time": use_sim_time},
            {"topic": watchdog_topic},
            {"timeout_sec": watchdog_timeout},
        ],
    )

    fake_odom_node = Node(
        package="go2_field_bringup",
        executable="fake_odom_publisher.py",
        name="fake_odom_publisher",
        condition=IfCondition(enable_fake_odom),
        parameters=[
            {"use_sim_time": use_sim_time},
            {"x": fake_odom_x},
            {"y": fake_odom_y},
            {"yaw": fake_odom_yaw},
        ],
        output="screen",
    )

    launch_actions = [
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("rviz_mode", default_value="scan2d"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=os.path.join(
                go2_field_config_share, "rviz", "hw_sensors_scan_view.rviz"
            ),
        ),
        DeclareLaunchArgument("enable_cmd_vel_bridge", default_value="false"),
        DeclareLaunchArgument("enable_keyboard_teleop", default_value="false"),
        DeclareLaunchArgument("enable_watchdog", default_value="true"),
        DeclareLaunchArgument("enable_scan_bridge", default_value="false"),
        DeclareLaunchArgument("enable_scan_boundary_filter", default_value="true"),
        DeclareLaunchArgument("enable_scan_tuning_gui", default_value="false"),
        DeclareLaunchArgument("enable_pointcloud_relay", default_value="true"),
        DeclareLaunchArgument("enable_joint_state_bridge", default_value="true"),
        DeclareLaunchArgument("enable_front_video_bridge", default_value="false"),
        DeclareLaunchArgument("enable_fake_odom", default_value="false"),
        DeclareLaunchArgument("sport_state_topic", default_value="/lf/sportmodestate"),
        DeclareLaunchArgument("lidar_cloud_topic", default_value="/utlidar/cloud"),
        DeclareLaunchArgument("lowstate_topic", default_value="/lowstate"),
        DeclareLaunchArgument("front_video_topic", default_value="/front_video_data"),
        DeclareLaunchArgument("front_image_topic", default_value="/front_camera/image_raw"),
        DeclareLaunchArgument("keyboard_cmd_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("keyboard_linear_speed", default_value="0.6"),
        DeclareLaunchArgument("keyboard_lateral_speed", default_value="0.6"),
        DeclareLaunchArgument("keyboard_angular_speed", default_value="1.0"),
        DeclareLaunchArgument("keyboard_key_timeout", default_value="0.25"),
        DeclareLaunchArgument("keyboard_publish_hz", default_value="20.0"),
        DeclareLaunchArgument("pointcloud_topic", default_value="/pointcloud"),
        DeclareLaunchArgument("pointcloud_frame_id", default_value="radar"),
        DeclareLaunchArgument("scan_target_frame", default_value="odom"),
        DeclareLaunchArgument("scan_min_height", default_value="-0.03"),
        DeclareLaunchArgument("scan_max_height", default_value="0.03"),
        DeclareLaunchArgument("scan_range_min", default_value="0.60"),
        DeclareLaunchArgument("scan_range_max", default_value="30.0"),
        DeclareLaunchArgument("scan_input_topic", default_value="/scan"),
        DeclareLaunchArgument("scan_output_topic", default_value="/scan_obstacles"),
        DeclareLaunchArgument(
            "scan_tuning_file",
            default_value=os.path.join(go2_field_config_share, "config", "scan_tuning.yaml"),
        ),
        DeclareLaunchArgument("fake_odom_x", default_value="0.0"),
        DeclareLaunchArgument("fake_odom_y", default_value="0.0"),
        DeclareLaunchArgument("fake_odom_yaw", default_value="0.0"),
        DeclareLaunchArgument("watchdog_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("watchdog_timeout", default_value="0.7"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        robot_description_launch,
        official_cmd_vel_bridge_node,
        keyboard_teleop_node,
        sport_state_bridge_node,
        lowstate_joint_state_bridge_node,
        pointcloud_relay_node,
        front_video_bridge_node,
        pointcloud_to_laserscan_node,
        scan_boundary_filter_node,
        scan_tuning_gui_node,
        watchdog_node,
        fake_odom_node,
    ]

    if rviz_launch is not None:
        launch_actions.append(rviz_launch)
    if rviz_node is not None:
        launch_actions.append(rviz_node)

    return LaunchDescription(launch_actions)
