import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
# from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, ThisLaunchFileDir
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import subprocess
import time

def launch_setup(context, *args, **kwargs):

    def check_ping(addr):
        try:
            res = subprocess.Popen(['ping', '-c1', addr], stdout=subprocess.PIPE)
            out, err = res.communicate()
            pingres = out.decode("cp949")
            i_recv = pingres.index('received')-2
            return int(pingres[i_recv])==1
        except Exception as e:
            print(f"[ERROR] PING TEST: {e}")
            return False

    moby_bringup_shared_path = get_package_share_directory('moby_bringup')
    moby_config_file = os.path.join(moby_bringup_shared_path, 'param', 'moby_config.yaml')
    with open(moby_config_file, 'r') as stream:
        stream.seek(0)
        moby_config = yaml.safe_load(stream)

    description_package = FindPackageShare('moby_description')
    moby_bringup_package = FindPackageShare('moby_bringup')

    # Initialize Arguments
    name = LaunchConfiguration("name")
    prefix = LaunchConfiguration("prefix")
    launch_rviz = LaunchConfiguration("launch_rviz")

    initial_joint_controllers = PathJoinSubstitution([moby_bringup_package, "controller", "indy_controllers_6dof.yaml"])

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
            moby_config["moby_type"],
            " ",
            "prefix:=",
            prefix,
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    front_lidar_ip = moby_config["lidar_ip"]["front"]
    rear_lidar_ip = moby_config["lidar_ip"]["rear"]
    while not (check_ping(front_lidar_ip) and check_ping(rear_lidar_ip)):
        time.sleep(1)

    '''
    Initialize 2 SICK lidar
    '''
    sick_scan_launch_file_path = os.path.join(moby_bringup_shared_path, 'launch/sick_tim_7xxS.launch')

    front_lidar_node_arguments = [sick_scan_launch_file_path,
                                  'hostname:=' + front_lidar_ip,
                                  'nodename:=front_lidar',
                                  'cloud_topic:=front_cloud',
                                  'frame_id:=front_lidar_link']
    front_lidar = Node(
        package="sick_scan",
        executable="sick_generic_caller",
        name='front_lidar',
        output="screen",
        arguments=front_lidar_node_arguments,
    )

    rear_lidar_node_arguments = [sick_scan_launch_file_path,
                                 'hostname:=' + rear_lidar_ip,
                                 'nodename:=rear_lidar',
                                 'cloud_topic:=rear_cloud',
                                 'frame_id:=rear_lidar_link'] # TODO:

    rear_lidar = Node(
        package="sick_scan",
        executable="sick_generic_caller",
        name='rear_lidar',
        output="screen",
        arguments=rear_lidar_node_arguments,
    )

    # laserscan_multi_merger is only used for amcl because it generates ghost point at each lidar's origin
    lidar_merger = Node(
        package="ira_laser_tools",
        executable="laserscan_multi_merger",
        output="screen",
        parameters=[
            {'destination_frame': "base_footprint"},
            {'scan_destination_topic': "/scan"},
            {'laserscan_topics': "front_lidar/scan rear_lidar/scan"},
        ],
    )


    '''
    Initialize REALSENSE camera
    '''
    realsense2_camera_package = FindPackageShare('realsense2_camera')
    rs_nodes = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [realsense2_camera_package, "/launch", "/rs_launch.py"]
            ),
            launch_arguments={
                "camera_name": cam_name+"_camera",
                "pointcloud.enable": "true",
                "serial_no": f"_{cam_serial}",
            }.items(),
        )
        for cam_name, cam_serial in moby_config["realsense_serial"].items()
    ]

    indy_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, initial_joint_controllers],
        output="screen",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    joint_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    )

    indy_driver = Node(
        package="moby_bringup",
        executable="indy_driver.py",
        name="indy_dcp",
        output="screen",
        emulate_tty=True,
        parameters=[
            {'indy_ip': moby_config["step_ip"]},
        ],
    )

    moby_driver = Node(
        package="moby_bringup",
        executable="moby_driver.py",
        name="moby_grpc",
        output="screen",
        parameters=[
            {'step_ip': moby_config["step_ip"],
             'use_gyro': moby_config["use_gyro"],
             'moby_type': moby_config["moby_type"],
             'body_length': moby_config['safety']['body']['length'],
             'body_width': moby_config['safety']['body']['width'],
             'lidar_margin': moby_config['safety']['lidar_margin'],
             'ir_margin': moby_config['safety']['ir_margin']
             },
        ],
    )

    ekf_param = PathJoinSubstitution(
        [moby_bringup_package, "param", "ekf.yaml"]
    )
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_param,
            # {'use_sim_time': false},
        ],
        remappings=[('/odometry/filtered', '/odom')],
    )

    rviz_config_file = PathJoinSubstitution(
        [moby_bringup_package, "rviz", "moby.rviz"]
    )
    rviz_node = Node(
        condition=IfCondition(launch_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # Delay start of robot_controller
    delay_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[joint_controller_spawner],
        )
    )

    # Delay rviz
    delay_rviz2_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_controller_spawner,
            on_exit=[rviz_node],
        )
    )

    joy_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(moby_bringup_shared_path,
                          'launch',
                          'moby_controller.launch.py')]),
        launch_arguments={
            'moby_type': moby_config["moby_type"]
        }.items(),
        condition=IfCondition(LaunchConfiguration("use_joypad")),
    )

    nodes_to_start = [
        # indy_driver, # TODO: enable when using Indy
        moby_driver,
        # indy_control_node,
        robot_state_publisher_node,
        # joint_state_broadcaster_spawner,
        # joint_controller_spawner,
        # delay_rviz2_spawner,
        rviz_node]

    nodes_to_start += [
        front_lidar,
        rear_lidar,
        lidar_merger
    ]

    nodes_to_start += rs_nodes
    nodes_to_start += [robot_localization_node]
    nodes_to_start += [joy_node]

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
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for multi-robot setup. \
            If changed than also joint names in the controllers configuration have to be updated."
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz", 
            default_value="false",
            description="set this value true to launch rviz (default: false)"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_joypad",
            default_value="true",
            description="set this value true not to launch joypad control node (default: true)"
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
