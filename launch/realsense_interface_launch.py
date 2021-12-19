from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
  return LaunchDescription([
    DeclareLaunchArgument('drone_id', default_value='drone0'),
    DeclareLaunchArgument('node_frequency', default_value='100.0'),
    Node(
      package='realsense_interface',
      executable='realsense_interface_node',
      name='realsense_interface',
      namespace=LaunchConfiguration('drone_id'),
      remappings=[
          ('realsense_interface/odom',   'self_localization/odom')
      ],
      output='screen',
      emulate_tty=True,
      parameters=[{'node_frequency': LaunchConfiguration('node_frequency')}]
    )
  ])
