import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Static Transform Publisher (world -> map)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']
        ),

        # RGBD Sync Node
        Node(
            package='rtabmap_sync',
            executable='rgbd_sync',
            name='rgbd_sync',
            namespace='rtabmap',
            output='screen',
            parameters=[
                {'approx_sync': True},
                {'queue_size': 10},
                {'depth_scale_factor': 1.0},
                {'depth_filter_z_min': 0.0},
                {'depth_filter_z_max': 5.0},
                {'use_sim_time': False},
            ],
            remappings=[
                ('rgb/image', '/front_camera/color/image_raw'),
                ('depth/image', '/front_camera/aligned_depth_to_color/image_raw'),
                ('rgb/camera_info', '/front_camera/color/camera_info'),
                ('rgbd_image', '/rtabmap/rgbd_image'),
            ],
        ),

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
                {'subscribe_depth': False},
                {'subscribe_rgbd': True},
                {'subscribe_scan': False},
                {'queue_size': 10},
                {'approx_sync': False},
                {'use_sim_time': False},
                # RTAB-Map Parameters
                {"RGBD/ProximityBySpace":"true"},
                {"RGBD/OptimizeFromGraphEnd":"false"},
                {"Kp/MaxDepth":"4.0"},
                {"Reg/Strategy":"0"},
                {"Icp/CorrespondenceRatio":"0.3"},
                {"Vis/MinInliers":"15"},
                {"Vis/InlierDistance":"0.1"},
                {"RGBD/AngularUpdate":"0.1"},
                {"RGBD/LinearUpdate":"0.1"},
                {"RGBD/ProximityPathMaxNeighbors" :"0"},
                {"Rtabmap/TimeThr":"0"},
                {"Mem/RehearsalSimilarity":"0.30"},
                {"Reg/Force3DoF":"true"},
                {"GridGlobal/MinSize":"20"},
                {"Grid/ObstacleFiltering":"true"},
                {"Grid/MinClusterSize":"5"},
                {"GridGlobal/OccupancyThr":"0.65"},
            ],
            remappings=[
                ('odom', '/odom'),
                ('rgbd_image', '/rtabmap/rgbd_image'),
            ],
            arguments=['--delete_db_on_start']
        ),
    ])
