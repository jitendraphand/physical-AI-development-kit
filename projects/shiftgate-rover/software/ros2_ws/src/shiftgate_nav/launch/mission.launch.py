from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim", default_value="true"),
        Node(
            package="shiftgate_nav",
            executable="mission",
            name="shiftgate_mission",
            output="screen",
            parameters=[{"use_sim": LaunchConfiguration("use_sim")}],
        ),
    ])
