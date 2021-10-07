from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense_interface',
            executable='realsense_interface_node',
            name='realsense_interface_node',
            remappings=[
                ('/drone0/rs_t265/odom',   '/drone0/self_localization/odom')
            ],
            output='screen'
        )
    ])