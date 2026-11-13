"""Gazebo Harmonic (ROS 2 Jazzy) launch.  Requires ros_gz_sim / ros_gz_bridge."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world = PathJoinSubstitution([FindPackageShare("shiftgate_gazebo"), "worlds", "shiftgate_harmonic.sdf"])
    urdf = PathJoinSubstitution([FindPackageShare("shiftgate_description"), "urdf", "shiftgate.urdf.xacro"])
    # Users on Jazzy typically:  ros2 launch ros_gz_sim gz_sim.launch.py gz_args:=-r <world>
    # plus a ros_gz_bridge parameter_bridge.  We start robot_state_publisher here and
    # document the gz launch in the BUILD_GUIDE; this file still gives a one-liner
    # for the description + a hint process.
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value=world),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{
                "use_sim_time": True,
                "robot_description": ParameterValue(Command(["xacro ", urdf]), value_type=str),
            }],
        ),
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=["-topic", "robot_description", "-name", "shiftgate",
                       "-x", "0.094", "-y", "0.094", "-z", "0.03"],
            output="screen",
        ),
    ])
