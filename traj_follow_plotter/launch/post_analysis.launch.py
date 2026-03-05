#!/usr/bin/env python3
"""
Launch file for post-processing trajectory analysis from rosbag.
Analyzes rosbag file and generates link padding recommendations.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    """動的にコマンドを構築（空の引数は渡さない）"""
    bag = LaunchConfiguration('bag').perform(context)
    urdf = LaunchConfiguration('urdf').perform(context)
    base_link = LaunchConfiguration('base_link').perform(context)
    output_dir = LaunchConfiguration('output_dir').perform(context)
    state_topic = LaunchConfiguration('state_topic').perform(context)
    
    # 基本コマンド
    cmd = [
        'ros2', 'run', 'traj_follow_plotter', 'post_analysis',
        '--bag', bag,
        '--urdf', urdf,
        '--base-link', base_link,
    ]
    
    # オプション引数（空でない場合のみ追加）
    if output_dir:
        cmd.extend(['--output-dir', output_dir])
    
    if state_topic:
        cmd.extend(['--state-topic', state_topic])
    
    post_analysis_process = ExecuteProcess(
        cmd=cmd,
        output='screen',
        shell=False,
        additional_env={'PYTHONUNBUFFERED': '1'}
    )
    
    return [post_analysis_process]


def generate_launch_description():
    # パッケージからURDFパスを取得
    urdf_path_default = PathJoinSubstitution([
        FindPackageShare('traj_follow_plotter'),
        'urdf',
        'zx200.urdf'
    ])
    
    # Declare arguments
    bag_arg = DeclareLaunchArgument(
        'bag',
        description='Path to rosbag2 directory (required)'
    )
    
    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value='',
        description='Output directory (default: bag_parent/analysis_output)'
    )
    
    urdf_path_arg = DeclareLaunchArgument(
        'urdf',
        default_value=urdf_path_default,
        description='Path to URDF file (required for link padding analysis)'
    )
    
    base_link_arg = DeclareLaunchArgument(
        'base_link',
        default_value='base_link',
        description='Base link name for FK (default: base_link)'
    )
    
    state_topic_arg = DeclareLaunchArgument(
        'state_topic',
        default_value='',
        description='Controller state topic name (auto-detect if not specified)'
    )

    return LaunchDescription([
        bag_arg,
        output_dir_arg,
        urdf_path_arg,
        base_link_arg,
        state_topic_arg,
        OpaqueFunction(function=launch_setup)
    ])
