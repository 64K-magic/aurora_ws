import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('slamware_nav2_bridge')
    default_params = os.path.join(pkg_share, 'config', 'bridge_params.yaml')

    params_file = LaunchConfiguration('params_file')
    auto_reloc = LaunchConfiguration('auto_relocalize_on_start')
    reloc_delay = LaunchConfiguration('relocalization_delay_sec')

    reloc_call = ExecuteProcess(
        condition=IfCondition(auto_reloc),
        cmd=[
            'ros2', 'service', 'call',
            '/slamware_ros_sdk_server_node/relocalization',
            'slamware_ros_sdk/srv/RelocalizationRequest',
            '{}',
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Bridge node parameter file'),
        DeclareLaunchArgument(
            'auto_relocalize_on_start',
            default_value='false',
            description='Call Slamware relocalization service once after delay'),
        DeclareLaunchArgument(
            'relocalization_delay_sec',
            default_value='8.0',
            description='Seconds to wait before auto relocalization (slamware must be up)'),
        Node(
            package='slamware_nav2_bridge',
            executable='map_odom_bridge_node',
            name='map_odom_bridge_node',
            output='screen',
            parameters=[params_file],
        ),
        TimerAction(
            period=reloc_delay,
            actions=[reloc_call],
        ),
    ])
