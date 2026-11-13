"""One-liner launch: description + (optional) gazebo + mission node."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _setup(context, *args, **kwargs):
    use_sim = LaunchConfiguration("use_sim").perform(context).lower() in ("true", "1")
    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare("shiftgate_description"), "launch", "rsp.launch.py"])
            ]),
            launch_arguments={"use_sim_time": "true" if use_sim else "false"}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare("shiftgate_nav"), "launch", "mission.launch.py"])
            ]),
            launch_arguments={"use_sim": LaunchConfiguration("use_sim")}.items(),
        ),
    ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim", default_value="true"),
        OpaqueFunction(function=_setup),
    ])
