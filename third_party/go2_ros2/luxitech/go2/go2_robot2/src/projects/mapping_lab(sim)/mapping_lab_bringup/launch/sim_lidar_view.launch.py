import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    robot_name = LaunchConfiguration("robot_name")
    lite = LaunchConfiguration("lite")
    world = LaunchConfiguration("world")
    world_init_x = LaunchConfiguration("world_init_x")
    world_init_y = LaunchConfiguration("world_init_y")
    world_init_z = LaunchConfiguration("world_init_z")
    world_init_heading = LaunchConfiguration("world_init_heading")
    paused = LaunchConfiguration("paused")
    auto_unpause = LaunchConfiguration("auto_unpause")
    unpause_delay = LaunchConfiguration("unpause_delay")
    enable_rviz = LaunchConfiguration("enable_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    luxi_go2_description_share = get_package_share_directory("luxi_go2_description")
    mapping_lab_config_share = get_package_share_directory("mapping_lab_config")
    mapping_lab_bringup_share = get_package_share_directory("mapping_lab_bringup")
    go2_config_share = get_package_share_directory("go2_config")
    workspace_root = os.path.abspath(
        os.path.join(mapping_lab_bringup_share, "..", "..", "..", "..")
    )
    turtlebot3_gazebo_assets = os.path.join(
        workspace_root,
        "src",
        "third_party",
        "turtlebot3_simulations",
        "turtlebot3_gazebo",
    )

    joints_config = os.path.join(go2_config_share, "config/joints/joints.yaml")
    links_config = os.path.join(go2_config_share, "config/links/links.yaml")
    gait_config = os.path.join(go2_config_share, "config/gait/gait.yaml")
    default_world_path = os.path.join(
        mapping_lab_config_share, "worlds", "turtlebot3_house_classic.world"
    )
    turtlebot3_models_path = os.path.join(turtlebot3_gazebo_assets, "models")
    default_model_path = os.path.join(
        luxi_go2_description_share, "xacro/go2_l1_sim.xacro"
    )
    default_rviz_path = os.path.join(mapping_lab_config_share, "rviz/lidar_view.rviz")

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("champ_bringup"),
                "launch",
                "bringup.launch.py",
            )
        ),
        launch_arguments={
            "description_path": default_model_path,
            "joints_map_path": joints_config,
            "links_map_path": links_config,
            "gait_config_path": gait_config,
            "use_sim_time": use_sim_time,
            "robot_name": robot_name,
            "gazebo": "true",
            "lite": lite,
            "rviz": "false",
            "joint_controller_topic": "joint_group_effort_controller/joint_trajectory",
            "hardware_connected": "false",
            "publish_foot_contacts": "true",
            "close_loop_odom": "true",
            "state_estimation": "false",
        }.items(),
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("champ_gazebo"),
                "launch",
                "gazebo.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "robot_name": robot_name,
            "world": world,
            "lite": lite,
            "world_init_x": world_init_x,
            "world_init_y": world_init_y,
            "world_init_z": world_init_z,
            "world_init_heading": world_init_heading,
            "paused": paused,
            "gui": gui,
            "headless": PythonExpression(["'", gui, "'.lower() == 'false'"]),
            "close_loop_odom": "true",
        }.items(),
    )

    auto_unpause_cmd = TimerAction(
        period=unpause_delay,
        actions=[
            ExecuteProcess(
                cmd=["ros2", "service", "call", "/unpause_physics", "std_srvs/srv/Empty", "{}"],
                output="screen",
                condition=IfCondition(auto_unpause),
            )
        ],
    )

    ground_truth_odom_bridge = Node(
        package="go2_config",
        executable="ground_truth_odom.py",
        name="ground_truth_odom_bridge",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(enable_rviz),
        output="screen",
    )

    gazebo_model_path = AppendEnvironmentVariable(
        "GAZEBO_MODEL_PATH",
        turtlebot3_models_path,
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("robot_name", default_value="go2"),
            DeclareLaunchArgument("lite", default_value="false"),
            DeclareLaunchArgument("world", default_value=default_world_path),
            DeclareLaunchArgument("world_init_x", default_value="1.5"),
            DeclareLaunchArgument("world_init_y", default_value="-2.0"),
            DeclareLaunchArgument("world_init_z", default_value="0.35"),
            DeclareLaunchArgument("world_init_heading", default_value="0.0"),
            DeclareLaunchArgument("paused", default_value="false"),
            DeclareLaunchArgument("auto_unpause", default_value="false"),
            DeclareLaunchArgument("unpause_delay", default_value="2.0"),
            DeclareLaunchArgument("enable_rviz", default_value="true"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_path),
            gazebo_model_path,
            bringup_launch,
            gazebo_launch,
            auto_unpause_cmd,
            ground_truth_odom_bridge,
            rviz_node,
        ]
    )
