import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('pilsbot_indicators'),
        'config',
        'params.yaml'
        )

    node=Node(
        package = 'pilsbot_indicators',
        name = 'pilsbot_indiator_bridge',
        executable = 'pilsbot_indiator_bridge',
        parameters = [config]
        #parameters = [{"key": "value"}]
    )
    ld.add_action(node)
    return ld