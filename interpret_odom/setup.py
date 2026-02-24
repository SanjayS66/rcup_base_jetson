from setuptools import find_packages, setup

package_name = 'interpret_odom'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
                ('share/' + package_name + '/launch', ['launch/navigation_support_nodes.launch.py','launch/hardware.launch.py', 'launch/mapping.launch.py', 'launch/nav2_bringup_with_collision_monitor.launch.py']),
        ('share/' + package_name + '/config', ['config/twist_mux.yaml', 'config/nav2_params.yaml', 'config/dead_wheel_w_imu.yaml', 'config/mapper_params_online_async.yaml', 'config/lidar_filter.yaml', 'config/localization_params_online_async.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yash',
    maintainer_email='yashc2025@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'tracking_dead_wheel_node = interpret_odom.tracking_dead_wheel_odom:main',
            'test_send_recieve_serial_json = interpret_odom.serial_data_send_recieve_test:main', 
            'serial_arbiter = interpret_odom.serial_arbiter:main',
            
            ],
    },
)
