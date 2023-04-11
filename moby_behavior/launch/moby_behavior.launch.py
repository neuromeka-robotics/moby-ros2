import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions.node import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    config_file = os.path.join(get_package_share_directory('moby_behavior'), 'param')

    moby_bt = LaunchConfiguration('moby_bt', default=config_file + '/moby_bt.xml')

    behavior_tree = Node(
        package='moby_behavior',
        executable='moby_behavior',
        parameters=[{'moby_bt': moby_bt}],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(behavior_tree)
    return ld
