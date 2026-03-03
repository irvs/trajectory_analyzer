#!/usr/bin/env python3
"""
Launch file for traj_follow_record action server with namespace support.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # パッケージからURDFパスを取得
    urdf_path_default = PathJoinSubstitution([
        FindPackageShare('traj_follow_plotter'),
        'urdf',
        'zx200.urdf'
    ])
    
    # Declare arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='zx200',
        description='Namespace for the nodes'
    )
    
    state_topic_arg = DeclareLaunchArgument(
        'state_topic',
        default_value='/zx200/upper_arm_controller/controller_state',
        description='Controller state topic to subscribe'
    )
    
    output_root_arg = DeclareLaunchArgument(
        'output_root',
        default_value='traj_data',
        description='Root directory for output files'
    )
    
    record_bag_arg = DeclareLaunchArgument(
        'record_bag_all',
        default_value='true',
        description='Record all topics with rosbag'
    )
    
    max_lag_arg = DeclareLaunchArgument(
        'max_lag_s',
        default_value='5.0',
        description='Maximum lag time in seconds for phase lag estimation'
    )
    
    lag_method_arg = DeclareLaunchArgument(
        'lag_method',
        default_value='frequency',
        description='Lag estimation method: correlation, dtw, frequency, polynomial, adaptive_kalman'
    )
    
    urdf_path_arg = DeclareLaunchArgument(
        'urdf_path',
        default_value=urdf_path_default,
        description='Path to URDF file for FK calculation'
    )

    # Node configuration
    traj_follow_record_node = Node(
        package='traj_follow_plotter',
        executable='plot',
        name='traj_follow_record',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[{
            'state_topic': LaunchConfiguration('state_topic'),
            'output_root': LaunchConfiguration('output_root'),
            'record_bag_all': LaunchConfiguration('record_bag_all'),
            'max_lag_s': LaunchConfiguration('max_lag_s'),
            'phase_use_velocity': False,
            'lag_method': LaunchConfiguration('lag_method'),
            'urdf_path': LaunchConfiguration('urdf_path'),
            'fk_base_link': 'base_link',
            'fk_tip_link': 'bucket_end_link',
        }],
    )

    return LaunchDescription([
        namespace_arg,
        state_topic_arg,
        output_root_arg,
        record_bag_arg,
        max_lag_arg,
        lag_method_arg,
        urdf_path_arg,
        traj_follow_record_node,
    ])
