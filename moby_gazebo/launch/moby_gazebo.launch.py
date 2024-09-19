import os

from launch import LaunchDescription
from launch.actions import  DeclareLaunchArgument, IncludeLaunchDescription, \
                            RegisterEventHandler, OpaqueFunction, AppendEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    description_package = FindPackageShare('moby_description')
    gazebo_package = FindPackageShare('moby_gazebo')
    
    moby_gazebo_pkg = get_package_share_directory('moby_gazebo')

    # Initialize Arguments
    name = LaunchConfiguration("name")
    moby_type = LaunchConfiguration("moby_type")
    prefix = LaunchConfiguration("prefix")
    launch_rviz = LaunchConfiguration("launch_rviz")
    world_file  = LaunchConfiguration("world_file")

    # Define the world file path
    world_file_path = FindPackageShare('moby_gazebo'), '/worlds/', world_file, '.world'
    
    initial_joint_controllers = PathJoinSubstitution(
        [gazebo_package, "controller", "moby_rp_controllers.yaml"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([description_package, "urdf", "moby.urdf.xacro"]),
            " ",
            "name:=",
            name,
            " ",
            "moby_type:=",
            moby_type,
            " ",
            "prefix:=",
            prefix,
            " ",
            "sim_gazebo:=true",
            " ",
            "simulation_controllers:=",
            initial_joint_controllers,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution(
        [description_package, "rviz_config", "moby.rviz"]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name='robot_state_publisher',
        output="screen",
        parameters=[{"use_sim_time": True}, robot_description],
    )
    
    controllers = [
        "joint_state_broadcaster",
        "traction_motor_controller",
        "rotation_motor_controller",
        "joint_trajectory_controller"
    ]
    
    controller_spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[controller, "-c", "/controller_manager"],
        )
        for controller in controllers
    ]

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]
        ),
        launch_arguments={'world': world_file_path}.items()
    )

    # Spawn robot
    gazebo_spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_moby",
        arguments=["-entity", "moby", "-topic", "robot_description"],
        output="screen",
    )
    
    swerve_drive_node = Node(
        package='moby_gazebo',
        executable='swerve_drive_controller.py',
        name='swerve_drive_controller',
        output='screen'
    )
    
    rviz_node = Node(
        condition=IfCondition(launch_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )
    
    # # Delay start joint_state_broadcaster
    # delay_joint_state_broadcaster_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=gazebo_spawn_robot,
    #         on_exit=[joint_state_broadcaster_spawner],
    #     )
    # )

    # # Delay rviz
    # delay_rviz2_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=joint_state_broadcaster_spawner,
    #         on_exit=[rviz_node],
    #     )
    # )

    nodes_to_start = [
        AppendEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(moby_gazebo_pkg, "models")),
        
        *controller_spawners,
        swerve_drive_node,
        gazebo,
        gazebo_spawn_robot,
        robot_state_publisher_node,        
        rviz_node
        
        # delay_joint_state_broadcaster_spawner,
        # delay_rviz2_spawner,
    ]

    return nodes_to_start


def generate_launch_description():
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
            default_value="moby_rp",
            description="Type of moby robot.",
            choices=["moby_rp", "moby_rp_v3"]
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "world_file",
            default_value="example",
            description="Name of the world_file.",
            # choices=["example"]
        )
    )
 
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for multi-robot setup. \
            If changed than also joint names in the controllers configuration have to be updated."
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz", 
            default_value="true", 
            description="Launch RViz?"
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
    