import os
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):

    # Initialize Arguments
    map = LaunchConfiguration("map").perform(context)
    moby_type = LaunchConfiguration("moby_type").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    launch_rviz = LaunchConfiguration("launch_rviz").perform(context)

    map_file_path = os.path.join(
        FindPackageShare('moby_navigation').find('moby_navigation'), 'map', f'{map}.yaml'
    )

    if use_sim_time == 'true':
        param_dir = os.path.join(
            FindPackageShare('moby_navigation').find('moby_navigation'), 'param', f'{moby_type}_gazebo.yaml'
        )
    else:
        param_dir = os.path.join(
            FindPackageShare('moby_navigation').find('moby_navigation'), 'param', f'{moby_type}.yaml'
        )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("nav2_bringup").find("nav2_bringup"), "/launch/bringup_launch.py"
        ]),
        launch_arguments={
            'map': map_file_path,
            'use_sim_time': use_sim_time,
            'params_file': param_dir
        }.items(),
    )

    return [nav2]


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "map",
            default_value="example",
            description="Name of map file to load."
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "moby_type",
            default_value="moby_rp",
            description="Type of moby robot.",
            choices=["moby_rp", "moby_rp_v3"]
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Set this value to true to launch RViz (default: true)"
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
