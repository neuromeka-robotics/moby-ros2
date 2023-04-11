import os
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    
    moby_config_file = os.path.join(get_package_share_directory('moby_bringup'), 'param', 'moby_config.yaml')
    with open(moby_config_file, 'r') as stream:
        stream.seek(0)
        moby_config = yaml.safe_load(stream)

    joy_linux = Node(
        package="joy_linux",
        executable="joy_linux_node",
        name="joy_linux",
        output="screen",
    )

    controller = Node(
        package="moby_bringup",
        executable="controller.py",
        name="console_controller",
        output="screen",
        parameters=[
            {'moby_type': moby_config["moby_type"]}
        ],
    )

    nodes_to_start = [
        joy_linux,
        controller,
    ]
    return nodes_to_start


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
