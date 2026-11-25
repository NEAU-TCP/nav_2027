from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("small_point_lio"), "config", "mid360.yaml"
            ]),
        ),
        Node(
            package="small_point_lio",
            executable="small_point_lio_node",
            name="small_point_lio",
            output="screen",
            parameters=[params_file],
        ),
    ])
