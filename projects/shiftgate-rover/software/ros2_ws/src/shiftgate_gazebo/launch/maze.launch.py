"""Gazebo Classic (ROS 2 Humble) launch for the ShiftGate arena."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world = PathJoinSubstitution([FindPackageShare("shiftgate_gazebo"), "worlds", "shiftgate.world"])
    urdf = PathJoinSubstitution([FindPackageShare("shiftgate_description"), "urdf", "shiftgate.urdf.xacro"])
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value=world),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])
            ]),
            launch_arguments={"world": LaunchConfiguration("world")}.items(),
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{
                "use_sim_time": True,
                "robot_description": ParameterValue(Command(["xacro ", urdf]), value_type=str),
            }],
        ),
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=["-topic", "robot_description", "-entity", "shiftgate",
                       "-x", "0.094", "-y", "0.094", "-z", "0.03"],
            output="screen",
        ),
    ])
