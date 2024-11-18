from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration  # Import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare arguments
    indy_ip_arg = DeclareLaunchArgument(
        'indy_ip', 
        default_value='',
        description='IP address of the Indy robot'
    )

    indy_type_arg = DeclareLaunchArgument(
        'indy_type', 
        default_value='indy7',
        description='Type of the Indy robot'
    )

    # Define the node to launch
    indy_driver_node = Node(
        package='moby_bringup',
        executable='indy_driver.py',
        name='indy_driver',
        output='screen',
        parameters=[
            {'indy_ip': LaunchConfiguration('indy_ip')},
            {'indy_type': LaunchConfiguration('indy_type')}
        ]
    )

    # Create and return the launch description
    return LaunchDescription([
        indy_ip_arg,
        indy_type_arg,
        indy_driver_node
    ])
