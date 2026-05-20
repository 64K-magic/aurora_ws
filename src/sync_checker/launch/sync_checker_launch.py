# sync_checker: 订阅 odom + LaserScan，发布 odom_path，并周期性统计 scan 与 odom 的 header.stamp 时间差。
#
# 默认话题与当前工作空间 SLAM 管线对齐说明：
# - scan_topic 默认 /scan：与 wheeltec_slam_toolbox（mapper_params_online_async.yaml 的 scan_topic）
#   及 ros2_lidar_2d_converter 输出一致，用于检查 SLAM Toolbox 使用的 2D 激光链路。
# - 若只调试 Slamware / Aurora 自带 2D 激光，请启动时传入：
#     scan_topic:=/slamware_ros_sdk_server_node/scan
# - odom_topic 仍默认 Slamware 里程计，与 online_async_launch 中 slam_toolbox 的 odom remap 一致；
#   若使用其他里程计源，请传入 odom_topic:=<你的话题>。
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    odom_topic = LaunchConfiguration('odom_topic', default='/slamware_ros_sdk_server_node/odom')
    scan_topic = LaunchConfiguration('scan_topic', default='/scan')
    path_topic = LaunchConfiguration('path_topic', default='odom_path')
    path_frame_id = LaunchConfiguration('path_frame_id', default='odom')
    max_path_length = LaunchConfiguration('max_path_length', default='2000')
    time_tolerance = LaunchConfiguration('time_tolerance', default='0.08')

    return LaunchDescription([
        DeclareLaunchArgument(
            'odom_topic',
            default_value=odom_topic,
            description=(
                'Input nav_msgs/Odometry topic. Default matches slam_toolbox odom remap to Slamware. '
                'Override if your odometry is published elsewhere.'
            ),
        ),
        DeclareLaunchArgument(
            'scan_topic',
            default_value=scan_topic,
            description=(
                'Input sensor_msgs/LaserScan topic. Default /scan matches SLAM Toolbox + '
                'ros2_lidar_2d_converter (RoboSense->2D). For Aurora 2D only: '
                'scan_topic:=/slamware_ros_sdk_server_node/scan'
            ),
        ),
        DeclareLaunchArgument(
            'path_topic',
            default_value=path_topic,
            description='Output nav_msgs/Path topic for RViz odom trajectory.',
        ),
        DeclareLaunchArgument(
            'path_frame_id',
            default_value=path_frame_id,
            description='Frame ID stamped on the published Path (sync_checker TF-transforms poses if needed).',
        ),
        DeclareLaunchArgument(
            'max_path_length',
            default_value=max_path_length,
            description='Maximum number of poses retained in the path buffer.',
        ),
        DeclareLaunchArgument(
            'time_tolerance',
            default_value=time_tolerance,
            description='Warn when |scan.header.stamp - nearest odom.header.stamp| exceeds this (seconds).',
        ),

        Node(
            package='sync_checker',
            executable='sync_checker',
            name='sync_checker',
            output='screen',
            parameters=[
                {'path_frame_id': path_frame_id},
                {'max_path_length': max_path_length},
                {'time_tolerance': time_tolerance},
            ],
            remappings=[
                ('odom', odom_topic),
                ('scan', scan_topic),
                ('odom_path', path_topic),
            ],
        ),
    ])
