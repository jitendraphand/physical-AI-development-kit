from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    urdf = PathJoinSubstitution([FindPackageShare("shiftgate_description"), "urdf", "shiftgate.urdf.xacro"])
    rviz = PathJoinSubstitution([FindPackageShare("shiftgate_description"), "rviz", "shiftgate.rviz"])
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "robot_description": ParameterValue(Command(["xacro ", urdf]), value_type=str),
            }],
        ),
        Node(package="joint_state_publisher", executable="joint_state_publisher", name="jsp"),
        Node(package="rviz2", executable="rviz2", arguments=["-d", rviz]),
    ])
