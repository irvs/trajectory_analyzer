#!/usr/bin/env python3
"""
静的なLink Correspondence可視化用のLaunchファイル
link_correspondence_nearest.csvを読み込んでRVizで表示
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('traj_follow_plotter')
    
    # Launch引数
    csv_dir_arg = DeclareLaunchArgument(
        'csv_dir',
        default_value='',
        description='Directory containing link_correspondence_nearest.csv'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_share, 'rviz', 'correspondence_viz.rviz'),
        description='RViz config file path'
    )
    
    # 可視化ノード
    visualize_node = Node(
        package='traj_follow_plotter',
        executable='visualize_correspondence_node',
        name='visualize_correspondence_node',
        output='screen',
        parameters=[{
            'csv_dir': LaunchConfiguration('csv_dir'),
        }]
    )
    
    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')]
    )
    
    return LaunchDescription([
        csv_dir_arg,
        rviz_config_arg,
        visualize_node,
        rviz_node,
    ])
