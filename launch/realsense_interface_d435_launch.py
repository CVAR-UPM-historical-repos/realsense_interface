from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, 
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def realsenseNode(context, *args, **kwargs):

    # Declare arguments
    namespace = DeclareLaunchArgument('namespace', default_value='drone0')
    node_frequency = DeclareLaunchArgument('node_frequency', default_value='100.0')
    device = DeclareLaunchArgument('device', default_value='')
    device_string = LaunchConfiguration('device')

    tf_device_config = PathJoinSubstitution([
        FindPackageShare('realsense_interface'),
        'config/tf_device',
        device_string+'.yaml'
    ])

    print(tf_device_config)

    return LaunchDescription([
        namespace,
        device,

        Node(
          package='realsense_interface',
          executable='realsense_interface_node',
          namespace=LaunchConfiguration('namespace'),
          parameters=[{'node_frequency': LaunchConfiguration('node_frequency')},
                      LaunchConfiguration('tf_device_config')],
          remappings=[
              ('sensor_measurements/realsense/odom',   'sensor_measurements/odom')
          ],
          output='screen',
          emulate_tty=True,
          # parameters=[{'node_frequency': LaunchConfiguration('node_frequency')}]
        )
  ])

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=realsenseNode)
    ])
    
    # Declare arguments
    namespace = DeclareLaunchArgument('namespace', default_value='drone0')
    node_frequency = DeclareLaunchArgument('node_frequency', default_value='100.0')
    device = DeclareLaunchArgument('device', default_value='')
    device_string = LaunchConfiguration('device')

    tf_device_config = PathJoinSubstitution([
        FindPackageShare('realsense_interface'),
        'config/tf_device',
        device_string+'.yaml'
    ])
    

    print(tf_device_config)

    return LaunchDescription([
        namespace,
        device,

        Node(
          package='realsense_interface',
          executable='realsense_interface_node',
          namespace=LaunchConfiguration('namespace'),
          parameters=[{'node_frequency': LaunchConfiguration('node_frequency')},
                      LaunchConfiguration('tf_device_config')],
          remappings=[
              ('sensor_measurements/realsense/odom',   'sensor_measurements/odom')
          ],
          output='screen',
          emulate_tty=True,
          # parameters=[{'node_frequency': LaunchConfiguration('node_frequency')}]
        )
  ])
