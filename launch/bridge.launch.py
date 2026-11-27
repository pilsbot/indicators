import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pilsbot_description import profile


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_profile', default_value='pilsbot',
            description='Robot profile: a name shipped with pilsbot_description, or a path.'),
        OpaqueFunction(function=launch_setup),
    ])


def launch_setup(context):
    robot_profile = profile.load(LaunchConfiguration('robot_profile').perform(context))
    config = os.path.join(
        get_package_share_directory('pilsbot_indicators'),
        'config',
        'params.yaml'
        )

    node=Node(
        package = 'pilsbot_indicators',
        name = 'pilsbot_indiator_bridge',
        executable = 'pilsbot_indiator_bridge',
        # The serial port comes from the robot profile.
        parameters = [config, {'serial_port': robot_profile.devices['lighting']}],
        # The bridge aborts when the lighting board's serial port is missing
        # (unplugged or not enumerated yet); start it again until it is there.
        # The StVO lights must come back without restarting the stack.
        respawn = True,
        respawn_delay = 5.0,
    )
    return [node]