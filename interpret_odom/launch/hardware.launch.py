import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # --- Paths to External Launch Files ---
    # 1. SLLidar A2M8 Launch
    sllidar_launch_path = os.path.join(
        get_package_share_directory('sllidar_ros2'), 
        'launch', 
        'view_sllidar_a2m8_launch.py'
    )
    
    # 2. Laser Scan Merger Launch
    merger_launch_path = os.path.join(
        get_package_share_directory('ros2_laser_scan_merger'), 
        'launch', 
        'merge_2_scan.launch.py'
    )

    # --- Directories for local config ---
    sll_dir = get_package_share_directory('sllidar_ros2')
    rviz_config_dir = os.path.join(sll_dir, 'rviz', 'sllidar_ros2_final.rviz')

    # --- Node Definitions ---
    zed_link = Node(
        package='interpret_odom', 
        executable='zed_dynamic_link',
        name='zed_dynamic_link',
        output='screen'
    )

    foxglove_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{'port': 8765, 'max_qos_depth': 1000}],
        ros_arguments=['--log-level', 'ERROR']
    )

    # --- Inclusion Actions ---
    include_sllidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sllidar_launch_path)
    )

    include_merger = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(merger_launch_path)
    )

    # --- Return Launch Description ---
    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        
        # External Launch Files
        include_sllidar,
        include_merger,
        
        # Your specific nodes
        zed_link, # Uncomment if needed
        
        TimerAction(
            period=3.0, 
            actions=[foxglove_node]
        )
    ])