#!/usr/bin/env python3
"""
Launch file for post-processing trajectory analysis.
Analyzes existing data.csv and generates plots with lag compensation.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # パッケージからURDFパスを取得
    urdf_path_default = PathJoinSubstitution([
        FindPackageShare('traj_follow_plotter'),
        'urdf',
        'zx200.urdf'
    ])
    
    # Declare arguments
    data_dir_arg = DeclareLaunchArgument(
        'data_dir',
        description='Directory containing data.csv and plan.csv (required)'
    )
    
    output_arg = DeclareLaunchArgument(
        'output',
        default_value='plot_reanalyzed.png',
        description='Output PNG filename (default: plot_reanalyzed.png)'
    )
    
    max_lag_arg = DeclareLaunchArgument(
        'max_lag',
        default_value='5.0',
        description='Maximum lag in seconds (default: 5.0)'
    )
    
    lag_method_arg = DeclareLaunchArgument(
        'lag_method',
        default_value='progress',
        description='Lag estimation method: correlation, frequency, progress, etc. (default: progress)'
    )
    
    no_save_compensated_arg = DeclareLaunchArgument(
        'no_save_compensated',
        default_value='false',
        description='Do NOT save compensated feedback data (default: false)'
    )
    
    urdf_path_arg = DeclareLaunchArgument(
        'urdf',
        default_value=urdf_path_default,
        description='Path to URDF file for FK-based EE position compensation'
    )
    
    base_link_arg = DeclareLaunchArgument(
        'base_link',
        default_value='base_link',
        description='Base link name for FK (default: base_link)'
    )
    
    tip_link_arg = DeclareLaunchArgument(
        'tip_link',
        default_value='bucket_end_link',
        description='Tip link name for FK (default: bucket_end_link)'
    )

    # コマンドライン引数を構築
    cmd = [
        'ros2', 'run', 'traj_follow_plotter', 'post_analysis',
        '--dir', LaunchConfiguration('data_dir'),
        '--output', LaunchConfiguration('output'),
        '--max-lag', LaunchConfiguration('max_lag'),
        '--lag-method', LaunchConfiguration('lag_method'),
        '--urdf', LaunchConfiguration('urdf'),
        '--base-link', LaunchConfiguration('base_link'),
        '--tip-link', LaunchConfiguration('tip_link'),
    ]
    
    # no_save_compensated が true の場合のみフラグを追加
    # NOTE: LaunchConfigurationは直接条件分岐できないため、
    # シンプルに常に引数を渡す形にするか、ユーザーがコマンドで制御する
    
    post_analysis_process = ExecuteProcess(
        cmd=cmd,
        output='screen',
        shell=False
    )

    return LaunchDescription([
        data_dir_arg,
        output_arg,
        max_lag_arg,
        lag_method_arg,
        no_save_compensated_arg,
        urdf_path_arg,
        base_link_arg,
        tip_link_arg,
        post_analysis_process,
    ])
