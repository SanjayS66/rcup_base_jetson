import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    sll_dir = get_package_share_directory('sllidar_ros2')

    filter_config_file = os.path.join(sll_dir, 'config', 'lidar_filter.yaml')

    rviz_config_dir = os.path.join(sll_dir, 'rviz', 'sllidar_ros2_final.rviz')

    channel_type = LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/lidar')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200')
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Boost')



    tf_footprint = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', 
                   '--frame-id', 'base_link', '--child-frame-id', 'base_footprint']
    )
    
    tf_laser = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['--x', '0.385', '--y', '0', '--z', '-0.027', '--yaw', '0', '--pitch', '0', '--roll', '0', 
                   '--frame-id', 'base_link', '--child-frame-id', 'laser']
    )

    filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_to_scan_filter_chain',
        parameters=[filter_config_file],
        remappings=[
            ('scan', 'scan_raw'), 
            ('scan_filtered', 'scan')
        ],
        output='screen'
    )

    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{
            'channel_type': channel_type,
            'serial_port': serial_port,
            'serial_baudrate': serial_baudrate,
            'frame_id': frame_id,
            'inverted': inverted,
            'angle_compensate': angle_compensate,
            'scan_mode': scan_mode,
            'queue_size': 10
        }],
        remappings=[('scan', 'scan_raw')],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2', executable='rviz2', 
        arguments=['-d', rviz_config_dir]
    )



    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        lidar_node,
        filter_node,
        tf_footprint,
        tf_laser,
        Node(

            package='foxglove_bridge',

            executable='foxglove_bridge',

            name='foxglove_bridge',

            parameters=[{

                'port': 8765,

            }]

        )                
#        rviz_node
    ])
