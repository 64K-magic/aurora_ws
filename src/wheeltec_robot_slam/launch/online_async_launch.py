from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess

def generate_launch_description():


    return LaunchDescription([

        launch_ros.actions.Node(
            parameters=[
        		get_package_share_directory("wheeltec_slam_toolbox") + '/config/mapper_params_online_async.yaml'
        	],
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            remappings=[('odom', '/slamware_ros_sdk_server_node/odom')],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.32', '0', '0.43', '0', '0', '0', 'base_link', 'rslidar'],
            name='static_transform_publisher_base_link_to_rslidar',
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'slamware_map', 'laser_map'],
            name='static_transform_publisher_slamware_map_to_laser_map',
        ),
    ])
