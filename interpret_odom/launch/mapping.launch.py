import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # --- 1. Directories & Configs ---
    sll_dir = get_package_share_directory('sllidar_ros2')
    slam_params_file = os.path.join(sll_dir, 'config', 'mapper_params_online_async.yaml')

    # --- 2. Arguments ---
    mode_arg = DeclareLaunchArgument(
        'mode', default_value='mapping',
        description='Options: "mapping" or "localization"'
    )
    
    map_file_arg = DeclareLaunchArgument(
        'map_file', default_value='/home/rm/rcup_testing_maps/mymap',
        description='Path to serialized map file (no extension) for localization'
    )

    # --- 3. Nodes ---
    
    # Mapping Node (Runs when mode='mapping')
    mapper_node = Node(
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('mode'), "' == 'mapping'"])),
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': False},
            {'mode': 'mapping'}
        ]
    )

    # Localization Node (Runs when mode='localization')
    localizer_node = Node(
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('mode'), "' == 'localization'"])),
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {
                'use_sim_time': False,
                'mode': 'localization',
                'map_file_name': LaunchConfiguration('map_file'),
		'map_start_pose': [0.0, 0.0, 0.0]  # Added quotes around the ke           
 }
        ]
    )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        mode_arg,
        map_file_arg,
        mapper_node,
        localizer_node,
    ])
