#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. Joy Node
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
    )

    # 2. Twist Mux Node
    twist_mux_params_path = os.path.join(
        get_package_share_directory('interpret_odom'),
        'config',
        'twist_mux.yaml'
    )

    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        arguments=[
            '--ros-args',
            '--params-file',
            twist_mux_params_path
        ],
    )

    # 3. Serial Motor Demo Node
    cmd_vel_to_pwm_node = Node(
        package='serial_motor_demo',
        executable='cmd_vel_to_pwm',
        name='cmd_vel_to_pwm',
        output='screen',
    )

    # 4. Serial Arbiter Node
    serial_arbiter_node = Node(
        package='interpret_odom',
        executable='serial_arbiter',
        name='serial_arbiter',
        output='screen'
    )

    # 5. EKF Node (Robot Localization)
    # This assumes dead_wheel_w_imu.yaml is installed to the config folder
    ekf_config_path = os.path.join(
        get_package_share_directory('interpret_odom'),
        'config',
        'dead_wheel_w_imu.yaml'
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path],
        # Optional: Remap topics if your config doesn't match your topic names
        # remappings=[('odometry/filtered', 'odom_filtered')] 
    )

    return LaunchDescription([
        joy_node,
        twist_mux_node,
        cmd_vel_to_pwm_node,
        serial_arbiter_node,
        ekf_node
    ])
