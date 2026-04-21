from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='laser_line_extraction_ros2',
            executable='line_extraction_node',
            name='line_extractor',
            parameters=[{
                # Node configuration
                'frequency': 30.0,            # Frequency of the processing loop [Hz]
                'frame_id': 'front_laser',          # Coordinate frame of the published lines [string]
                'scan_topic': 'front_scan_filtered',      # Topic name to subscribe to for LaserScan [string]
                'publish_markers': True,       # Enable/disable visualization_msgs/Marker output [bool]

                # Sensor uncertainty parameters
                'bearing_std_dev': 1e-5,       # Standard deviation of laser beam bearing [rad] [cite: 1]
                'range_std_dev': 0.012,        # Standard deviation of laser range measurement [m] [cite: 1]

                # Least squares fitting convergence thresholds
                'least_sq_angle_thresh': 0.0001,  # Change in angle to stop iterations [rad] [cite: 1]
                'least_sq_radius_thresh': 0.0001, # Change in radius to stop iterations [m] [cite: 1]

                # Geometric extraction constraints
                'max_line_gap': 0.5,           # Max distance between points to stay in the same line [m] [cite: 1]
                'min_line_length': 0.3,        # Minimum length for a line to be valid [m] [cite: 1]
                'min_range': 0.05,              # Ignore points closer than this distance [m] [cite: 1]
                'max_range': 250.0,            # Ignore points farther than this distance [m] [cite: 1]
                'min_split_dist': 0.04,        # Max distance a point can be from a line before splitting [m] [cite: 1]
                'outlier_dist': 0.06,          # Distance threshold to identify noisy outlier points [m] 
                'min_line_points': 10          # Minimum number of points required to form a line [count] 
            }]
        )
    ])