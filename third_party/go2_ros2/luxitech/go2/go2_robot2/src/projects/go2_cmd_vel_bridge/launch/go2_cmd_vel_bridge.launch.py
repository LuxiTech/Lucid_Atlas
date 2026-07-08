from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("request_topic", default_value="/api/sport/request"),
            DeclareLaunchArgument("expected_sender_ip", default_value="192.168.123.51"),
            DeclareLaunchArgument("max_linear_x", default_value="0.5"),
            DeclareLaunchArgument("max_linear_y", default_value="0.5"),
            DeclareLaunchArgument("max_angular_z", default_value="1.0"),
            DeclareLaunchArgument("cmd_timeout_sec", default_value="0.2"),
            DeclareLaunchArgument("stop_burst_count", default_value="12"),
            DeclareLaunchArgument("qos_depth", default_value="1"),
            DeclareLaunchArgument("desired_motion_mode", default_value="1"),
            DeclareLaunchArgument("desired_gait_type", default_value="1"),
            DeclareLaunchArgument("enforce_desired_motion_mode", default_value="true"),
            Node(
                package="go2_cmd_vel_bridge",
                executable="go2_cmd_vel_bridge_node",
                name="go2_cmd_vel_bridge",
                output="screen",
                parameters=[
                    {
                        "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                        "request_topic": LaunchConfiguration("request_topic"),
                        "expected_sender_ip": LaunchConfiguration("expected_sender_ip"),
                        "max_linear_x": ParameterValue(
                            LaunchConfiguration("max_linear_x"), value_type=float
                        ),
                        "max_linear_y": ParameterValue(
                            LaunchConfiguration("max_linear_y"), value_type=float
                        ),
                        "max_angular_z": ParameterValue(
                            LaunchConfiguration("max_angular_z"), value_type=float
                        ),
                        "cmd_timeout_sec": ParameterValue(
                            LaunchConfiguration("cmd_timeout_sec"), value_type=float
                        ),
                        "stop_burst_count": ParameterValue(
                            LaunchConfiguration("stop_burst_count"), value_type=int
                        ),
                        "qos_depth": ParameterValue(
                            LaunchConfiguration("qos_depth"), value_type=int
                        ),
                        "desired_motion_mode": ParameterValue(
                            LaunchConfiguration("desired_motion_mode"), value_type=int
                        ),
                        "desired_gait_type": ParameterValue(
                            LaunchConfiguration("desired_gait_type"), value_type=int
                        ),
                        "enforce_desired_motion_mode": ParameterValue(
                            LaunchConfiguration("enforce_desired_motion_mode"), value_type=bool
                        ),
                    }
                ],
            ),
        ]
    )
