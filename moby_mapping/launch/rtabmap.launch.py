import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([   
        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']),
                
        # RTAB-Map Node
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            namespace='rtabmap',
            output='screen',
            parameters=[
                {'frame_id': 'base_footprint'},
                {'odom_frame_id': 'odom'},
                {'subscribe_depth': True},
                {'subscribe_scan': True},
                {'queue_size': 10},
                {'approx_sync': True},
                {'use_sim_time': True},
                # RTAB-Map's parameters
                {'RGBD/ProximityBySpace': 'false'},
                {'RGBD/AngularUpdate': '0.01'},
                {'RGBD/LinearUpdate': '0.01'},
                {'RGBD/OptimizeFromGraphEnd': 'false'},
                {'Reg/Force3DoF': 'true'},
                {'Vis/MinInliers': '12'},
            ],
            remappings=[
                ('odom', '/odom'),
                ('scan', '/front_lidar/scan'),
                ('rgb/image', '/front_camera/image_raw'),
                ('depth/image', '/front_camera/depth/image_raw'), #/front_camera/points
                ('rgb/camera_info', '/front_camera/camera_info'),
            ],
            arguments=['--delete_db_on_start']
        ),
    ])
