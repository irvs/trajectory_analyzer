"""
動画生成用launchファイル - 1つで全て完結

使い方:
  # 仮想ディスプレイで録画（デフォルト）
  ros2 launch traj_follow_plotter video_generation.launch.py data_dir:=/path/to/run_YYYYMMDD_HHMMSS

  # 通常ディスプレイで表示
  ros2 launch traj_follow_plotter video_generation.launch.py data_dir:=/path/to/run_YYYYMMDD_HHMMSS use_xvfb:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, RegisterEventHandler, EmitEvent, OpaqueFunction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_nodes(context, *args, **kwargs):
    """条件に応じてノードを生成"""
    data_dir = LaunchConfiguration('data_dir').perform(context)
    use_xvfb = LaunchConfiguration('use_xvfb').perform(context)
    
    # use_xvfbがtrueならXvfbを使用、falseなら現在のDISPLAYを使用
    display = ':99' if use_xvfb.lower() == 'true' else os.environ.get('DISPLAY', ':0')
    
    # URDFとファイルパス
    urdf_file = PathJoinSubstitution([
        FindPackageShare('traj_follow_plotter'),
        'meshes',
        'zx200_video.xacro'
    ])
    csv_file = os.path.join(data_dir, 'data.csv')
    output_video = os.path.join(data_dir, 'animation.mp4')
    
    nodes = []
    
    # 1. Xvfb起動（use_xvfb=trueの場合のみ）
    if use_xvfb.lower() == 'true':
        nodes.append(
            ExecuteProcess(
                cmd=['Xvfb', ':99', '-screen', '0', '1920x1080x24'],
                output='screen',
                name='xvfb'
            )
        )
    
    # 2. Static TF: world → ref/base_link と world → fb/base_link
    # 両方のロボットを同じ位置に配置（重ねて表示）
    static_tf_ref = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_ref',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'ref/base_link'],
        output='screen'
    )
    nodes.append(static_tf_ref)
    
    static_tf_fb = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_fb',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'fb/base_link'],
        output='screen'
    )
    nodes.append(static_tf_fb)
    
    # 3. robot_state_publisher (Reference用) - xacroにprefix引数を渡す
    robot_state_publisher_ref = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='ref',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file.perform(context), ' prefix:=ref/']),
        }],
        remappings=[('joint_states', '/video_gen/joint_states_ref')],
        additional_env={'DISPLAY': display}
    )
    nodes.append(robot_state_publisher_ref)
    
    # 4. robot_state_publisher (Feedback用) - xacroにprefix引数を渡す
    robot_state_publisher_fb = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='fb',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file.perform(context), ' prefix:=fb/']),
        }],
        remappings=[('joint_states', '/video_gen/joint_states_fb')],
        additional_env={'DISPLAY': display}
    )
    nodes.append(robot_state_publisher_fb)
    
    # 5. video_player
    video_player = Node(
        package='traj_follow_plotter',
        executable='video_player',
        name='video_player_node',
        output='screen',
        parameters=[{
            'csv_path': csv_file,
            'playback_speed': 1.0,
            'loop': False if use_xvfb.lower() == 'true' else True  # 通常ディスプレイではループ再生
        }],
        additional_env={'DISPLAY': display}
    )
    
    video_player_delayed = TimerAction(
        period=3.0,
        actions=[video_player]
    )
    nodes.append(video_player_delayed)
    
    # 6. RViz（3秒後に起動）
    rviz = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', os.path.join(
                    FindPackageShare('traj_follow_plotter').perform(context),
                    'config',
                    'video.rviz'
                )],
                additional_env={'DISPLAY': display}
            )
        ]
    )
    nodes.append(rviz)
    
    # 7. ffmpeg（3秒後に録画開始、use_xvfb=trueの場合のみ）
    if use_xvfb.lower() == 'true':
        ffmpeg = TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ffmpeg', '-y',
                        '-f', 'x11grab',
                        '-video_size', '1920x1080',
                        '-framerate', '30',
                        '-i', display,
                        '-c:v', 'libx264',
                        '-preset', 'fast',
                        '-pix_fmt', 'yuv420p',
                        output_video
                    ],
                    output='screen',
                    name='ffmpeg',
                    additional_env={'DISPLAY': display}
                )
            ]
        )
        nodes.append(ffmpeg)
    
    # 8. video_playerが終了したら3秒待ってシャットダウン（use_xvfb=trueの場合のみ）
    if use_xvfb.lower() == 'true':
        shutdown_handler = RegisterEventHandler(
            OnProcessExit(
                target_action=video_player,
                on_exit=[
                    TimerAction(
                        period=3.0,
                        actions=[EmitEvent(event=Shutdown())]
                    )
                ]
            )
        )
        nodes.append(shutdown_handler)
    
    return nodes


def generate_launch_description():
    # 引数
    data_dir_arg = DeclareLaunchArgument(
        'data_dir',
        description='Path to recorded data directory'
    )
    
    use_xvfb_arg = DeclareLaunchArgument(
        'use_xvfb',
        default_value='true',
        description='Use Xvfb (virtual display) for recording. Set to false to use current display.'
    )
    
    return LaunchDescription([
        data_dir_arg,
        use_xvfb_arg,
        OpaqueFunction(function=generate_nodes)
    ])
