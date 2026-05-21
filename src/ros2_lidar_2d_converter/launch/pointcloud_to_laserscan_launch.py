from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    rsbringup_dir = get_package_share_directory('rslidar_sdk')
    rslaunch_dir = os.path.join(rsbringup_dir, 'launch')
    wheeltec_lidar3D = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(rslaunch_dir, 'start.py')),
    )

    return LaunchDescription([
        wheeltec_lidar3D,
        DeclareLaunchArgument(
            name='scanner', default_value='scanner',
            description='Namespace for sample topics'
        ),
        Node(
            package='ros2_lidar_2d_converter', executable='pointcloud_to_laserscan_node',
            remappings=[('rslidar_points', '/rslidar_points'),
                        ('scan', '/scan')],
            parameters=[{
                'target_frame': 'rslidar',
                'transform_tolerance': 0.35,
                'min_height': 0.0,
                'max_height': 0.6,
                'angle_min': -3.14159,  # -M_PI/2
                'angle_max': 3.14159,  # M_PI/2
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.1,
                'range_min': 0.25,
                'range_max': 30.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan'
        )
    ])
