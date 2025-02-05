from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.substitutions import ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


from launch.actions import TimerAction


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('pilsbot_indicators'),
        'config',
        'params.yaml'
        )

    included_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/bridge.launch.py'])
        # launch_arguments={
        #     "serial_port" : LaunchConfiguration("serial_port"),
        # }.items()
    )
    ld.add_action(included_bridge_launch)

    stvo_translator=Node(
        package = 'pilsbot_indicators',
        name = 'pilsbot_stvo_translation',
        executable = 'pilsbot_stvo_translation',
        parameters = [config],
        respawn=True, # might throw because something something reverse ion thrusters https://github.com/ros2/rcl/issues/1118
        respawn_delay=4,
        #parameters = [{"key": "value"}]
    )

    # launch it delayed, because it seems that it needs the existing topics
    timed_stvo_translator = TimerAction(period=4.0, actions=[stvo_translator])

    ld.add_action(timed_stvo_translator)
    return ld