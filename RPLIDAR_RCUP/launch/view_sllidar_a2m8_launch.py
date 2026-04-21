#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import TimerAction # <-- Add this line

def generate_launch_description():
    channel_type =  LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200')
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Standard')

    rviz_config_dir = os.path.join(
            get_package_share_directory('sllidar_ros2'),
            'rviz',
            'sllidar_ros2.rviz')

    front_filter_config_file = os.path.join(
            get_package_share_directory('sllidar_ros2'),
            'config',
            'front_lidar_filter.yaml')
    
    rear_filter_config_file = os.path.join(
            get_package_share_directory('sllidar_ros2'),
            'config',
            'rear_lidar_filter.yaml')
    
    return LaunchDescription([


        DeclareLaunchArgument(
            'channel_type',
            default_value=channel_type,
            description='Specifying channel type of lidar'),

        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='Specifying usb port to connected lidar'),

        DeclareLaunchArgument(
            'serial_baudrate',
            default_value=serial_baudrate,
            description='Specifying usb port baudrate to connected lidar'),
        
        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar'),

        DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='Specifying whether or not to invert scan data'),

        DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='Specifying whether or not to enable angle_compensate of scan data'),

        DeclareLaunchArgument(
            'scan_mode',
            default_value=scan_mode,
            description='Specifying scan mode of lidar'),


                Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_to_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_front_laser',
            arguments=['0.370', '0', '-0.035', '0', '0', '0', 'base_link', 'front_laser']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_rear_laser',
            arguments=['-0.38959', '0', '-0.05576', '0.0', '0', '0', 'base_link', 'rear_laser']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser']
        ),

        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='front_lidar',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': '/dev/front_lidar',
                'serial_baudrate': 115200,
                'frame_id': 'front_laser',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Standard'
            }],
            remappings=[
                ('/scan', '/front_scan')
            ],
            output='screen'
        ),

        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='sllidar_ros2',
                    executable='sllidar_node',
                    name='rear_lidar',
                    parameters=[{
                        'channel_type': 'serial',
                        'serial_port': '/dev/rear_lidar',
                        'serial_baudrate': 115200,
                        'frame_id': 'rear_laser',
                        'inverted': True,
                        'angle_compensate': True,
                        'scan_mode': 'Standard'
                    }],
                    remappings=[
                        ('/scan', '/rear_scan')
                    ],
                    output='screen'
                )
            ]
        ),

        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='front_lidar_filter',              # Matches the YAML root name
            parameters=[front_filter_config_file],
            remappings=[
                ('/scan', '/front_scan'),                  # Subscribes to raw front scan
                ('/scan_filtered', '/front_scan_filtered') # Publishes the 180-deg front scan
            ],
            output='screen'
        ),

        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='rear_lidar_filter',               # Matches the YAML root name
            parameters=[rear_filter_config_file],
            remappings=[
                ('/scan', '/rear_scan'),                   # Subscribes to raw rear scan
                ('/scan_filtered', '/rear_scan_filtered')  # Publishes the 180-deg rear scan
            ],
            output='screen'
        ),


    ])