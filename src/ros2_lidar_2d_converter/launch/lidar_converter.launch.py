from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_lidar_2d_converter',
            executable='lidar_2d_converter',
            name='lidar_2d_converter',
            output='screen',
            parameters=[{
                'output_frame': 'map',
                'min_height': 0.3,
                'max_height': 0.757,
                'angle_min': -3.14159,  # -M_PI
                'angle_max': 3.14159,   # M_PI
                'angle_increment': 0.0174533,  # ~1 degree
                'scan_time': 0.1,
                'range_min': 0.1,
                'range_max': 10.0
            }],
            remappings=[('rsscan','scan')]
        )
    ])