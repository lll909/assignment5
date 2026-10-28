"""Display an existing global PCD and its 2D projection without replaying a bag."""
from datetime import datetime
from pathlib import Path
import math
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def setup(context):
    def value(name):
        from launch.substitutions import LaunchConfiguration
        return LaunchConfiguration(name).perform(context)
    pcd = Path(value('pcd_path')).expanduser().resolve()
    if not pcd.is_file():
        raise RuntimeError(f'PCD not found: {pcd}')
    frame = value('map_frame')
    voxel, resolution = float(value('voxel_size')), float(value('grid_resolution'))
    lower, upper = float(value('height_min')), float(value('height_max'))
    period = int(value('publishing_period_ms'))
    if (not frame or not all(math.isfinite(v) for v in (voxel, resolution, lower, upper))
            or voxel <= 0 or resolution <= 0 or lower >= upper or period < 1000):
        raise RuntimeError('Invalid frame, voxel/resolution, height range or period (<1000 ms)')
    share = Path(get_package_share_directory('mid360_mapping'))
    nodes = [
        Node(package='pcl_ros', executable='pcd_to_pointcloud', name='pcd_publisher',
             output='screen', parameters=[{'file_name': str(pcd), 'tf_frame': frame,
                 'publishing_period_ms': period, 'use_sim_time': False}],
             remappings=[('cloud_pcd', '/pcd_full_map')]),
        Node(package='mid360_mapping', executable='map_processor_node',
             name='map_processor_node', output='screen', parameters=[{
                 'use_sim_time': False, 'input_topic': '/pcd_full_map', 'map_frame': frame,
                 'output_directory': str(Path(value('output_directory')).expanduser().resolve()),
                 'voxel_size': voxel, 'grid_resolution': resolution,
                 'obstacle_height_min': lower, 'obstacle_height_max': upper,
                 'remove_outliers': value('remove_outliers') == 'true',
                 'outlier_mean_k': 20, 'outlier_stddev': 1.0,
                 'overwrite_existing': False}]),
    ]
    if value('rviz') == 'true':
        nodes.append(Node(package='rviz2', executable='rviz2', output='screen',
                          arguments=['-d', str(share / 'rviz/pcd_alignment.rviz')],
                          parameters=[{'use_sim_time': False}]))
    return nodes


def generate_launch_description():
    defaults = {
        'pcd_path': str(Path.home() / 'ws_point_lio/PCD/scans.pcd'),
        'map_frame': 'camera_init',
        'output_directory': str(Path.cwd() / ('maps_preview_' + datetime.now().strftime('%Y%m%d_%H%M%S'))),
        'voxel_size': '0.10', 'grid_resolution': '0.10',
        'height_min': '-10.0', 'height_max': '10.0',
        'publishing_period_ms': '10000',
    }
    actions = [DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
    actions += [DeclareLaunchArgument('remove_outliers', default_value='false', choices=['true', 'false']),
                DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
                OpaqueFunction(function=setup)]
    return LaunchDescription(actions)
