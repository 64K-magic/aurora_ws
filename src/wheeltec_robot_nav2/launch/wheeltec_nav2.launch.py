import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
        
    wheeltec_nav_dir = get_package_share_directory('wheeltec_nav2')
    wheeltec_nav_launchr = os.path.join(wheeltec_nav_dir, 'launch')


    map_dir = os.path.join(wheeltec_nav_dir, 'map')
    map_file = LaunchConfiguration('map', default=os.path.join(
        map_dir, 'WHEELTEC.yaml'))

    param_dir = os.path.join(wheeltec_nav_dir, 'param','wheeltec_params')
    param_file = LaunchConfiguration('params', default=os.path.join(
    param_dir, 'param_top_omni.yaml'))

    bridge_share = get_package_share_directory('slamware_nav2_bridge')
    auto_reloc = LaunchConfiguration('auto_relocalize_on_start', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'auto_relocalize_on_start',
            default_value='false',
            description='Call Slamware relocalization once when map_odom bridge starts'),
        DeclareLaunchArgument(
            'map',
            default_value=map_file,
            description=(
                '地图 YAML 路径（PGM+YAML）。'
            ),
        ),

        DeclareLaunchArgument(
            'params',
            default_value=param_file,
            description='Full path to param file to load'),
        Node(
            name='waypoint_cycle',
            package='nav2_waypoint_cycle',
            executable='nav2_waypoint_cycle',
        ),
        # /scan 的 frame_id 为 rslidar（见 pointcloud_to_laserscan_launch.py）
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.32', '0', '0.43', '0', '0', '0', 'base_link', 'rslidar'],
            name='static_tf_base_link_to_rslidar',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'slamware_map', 'map'],
            name='static_tf_slamware_map_to_map',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bridge_share, 'launch', 'slamware_nav2_bridge.launch.py')),
            launch_arguments={
                'auto_relocalize_on_start': auto_reloc,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [wheeltec_nav_launchr, '/bringup_launch.py']),
            launch_arguments={
                'map': map_file,
                'use_sim_time': use_sim_time,
                'params_file': param_file,
                # 须为 Python 布尔字面量 True/False：bringup 里 PythonExpression(['not ', slam]) 会 eval 拼接结果
                'slam': 'False',
                'use_slamware_loc': 'True',
            }.items(),
        ),
    ])
