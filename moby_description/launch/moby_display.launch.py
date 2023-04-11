from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "name",
            default_value="moby"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "moby_type",
            # default_value="moby_rp",
            description="Type of Moby robot.",
            choices=["moby_rp", "moby_agri"]
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
            multi-robot setup. If changed than also joint names in the controllers configuration \
            have to be updated."
        )
    )

    # Initialize Arguments
    name = LaunchConfiguration("name")
    moby_type = LaunchConfiguration("moby_type")
    prefix = LaunchConfiguration("prefix")

    description_package = FindPackageShare('moby_description')

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([description_package, "urdf", 'moby.urdf.xacro']),
            " ",
            "name:=",
            name,
            " ",
            "moby_type:=",
            moby_type,
            " ",
            "prefix:=",
            prefix
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution(
        [description_package, "rviz", "moby.rviz"]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file]
    )

    nodes = [
        # joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
