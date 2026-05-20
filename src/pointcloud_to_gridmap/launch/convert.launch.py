import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('pointcloud_to_gridmap')
    default_params_file = os.path.join(pkg_dir, 'config', 'params.yaml')
    
    # 声明参数
    args = [
        DeclareLaunchArgument('params_file', default_value=default_params_file,
                              description='Parameter file (YAML)'),
        DeclareLaunchArgument('pointcloud_file', default_value='',
                              description='Override input point cloud file path'),
        DeclareLaunchArgument('output_dir', default_value='',
                              description='Override output directory'),
        DeclareLaunchArgument('z_min', default_value='',
                              description='Override z_min (meters)'),
        DeclareLaunchArgument('z_max', default_value='',
                              description='Override z_max (meters)'),
        DeclareLaunchArgument('map_resolution', default_value='',
                              description='Override resolution (m/px)'),
        DeclareLaunchArgument('occupied_threshold', default_value='',
                              description='Override occupied threshold'),
    ]
    
    def launch_setup(context, *args, **kwargs):
        # 获取所有覆盖参数
        overrides = {}
        for arg_name in ['pointcloud_file', 'output_dir', 'z_min', 'z_max', 
                         'map_resolution', 'occupied_threshold']:
            val = LaunchConfiguration(arg_name).perform(context)
            if val.strip():  # 如果非空字符串则覆盖
                overrides[arg_name] = val
        
        # 构建 parameters 列表：先参数文件，再覆盖字典
        params = [LaunchConfiguration('params_file')]
        if overrides:
            params.append(overrides)
        
        node = Node(
            package='pointcloud_to_gridmap',
            executable='pointcloud_to_gridmap_node',
            name='pointcloud_to_gridmap_node',
            output='screen',
            parameters=params
        )
        return [node]
    
    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])