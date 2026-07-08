from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare("luxi_hikrobot_cu013_camera"),
        "config",
        "cu013.yaml",
    ])

    rviz_config = PathJoinSubstitution([
        FindPackageShare("luxi_hikrobot_cu013_camera"),
        "rviz",
        "cu013_camera.rviz",
    ])

    use_rviz = LaunchConfiguration("rviz")
    external_trigger_enabled = LaunchConfiguration("external_trigger_enabled")
    trigger_source = LaunchConfiguration("trigger_source")
    trigger_activation = LaunchConfiguration("trigger_activation")

    camera_node = Node(
        package="luxi_hikrobot_cu013_camera",
        executable="hikrobot_cu013_camera_node",
        name="hikrobot_cu013_camera_node",
        namespace="hikrobot/cu013",
        output="screen",
        parameters=[
            config_file,
            {
                "external_trigger_enabled": external_trigger_enabled,
                "trigger_source": trigger_source,
                "trigger_activation": trigger_activation,
            },
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("external_trigger_enabled", default_value="false"),
        DeclareLaunchArgument("trigger_source", default_value="Line0"),
        DeclareLaunchArgument("trigger_activation", default_value="RisingEdge"),
        camera_node,
        rviz_node,
    ])
