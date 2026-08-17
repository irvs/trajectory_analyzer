"""
動画生成用launchファイル - 1つで全て完結（2台のバックホウ表示）

ref (青): Plan目標値
fb (緑): Feedback実測値

使い方:
  # 仮想ディスプレイで録画（デフォルト、等速再生）
  ros2 launch traj_follow_plotter video_generation.launch.py data_dir:=/path/to/run_YYYYMMDD_HHMMSS

  # 複数のrunディレクトリを指定して録画（カンマ区切りまたはスペース区切り）
  ros2 launch traj_follow_plotter video_generation.launch.py data_dir:=/path/to/run1,/path/to/run2

  # 2倍速で録画
  ros2 launch traj_follow_plotter video_generation.launch.py data_dir:=/path/to/run_YYYYMMDD_HHMMSS playback_speed:=2.0

  # 0.5倍速（スロー再生）で録画
  ros2 launch traj_follow_plotter video_generation.launch.py data_dir:=/path/to/run_YYYYMMDD_HHMMSS playback_speed:=0.5

  # 通常ディスプレイで表示（等速、録画なし）
  ros2 launch traj_follow_plotter video_generation.launch.py data_dir:=/path/to/run_YYYYMMDD_HHMMSS record:=false

  # 通常ディスプレイで2倍速表示
  ros2 launch traj_follow_plotter video_generation.launch.py data_dir:=/path/to/run_YYYYMMDD_HHMMSS record:=false playback_speed:=2.0

  # namespace付きで起動（複数ロボット表示用、別ターミナルで実行）
  ros2 launch traj_follow_plotter video_generation.launch.py data_dir:=/path/to/run1 robot_namespace:=robot1 use_xvfb:=false
  # 別ターミナルで
  ros2 launch traj_follow_plotter video_generation.launch.py data_dir:=/path/to/run2 robot_namespace:=robot2 use_xvfb:=false
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
import tempfile


def parse_data_dirs(data_dir_str):
    """カンマやスペース区切りの文字列からディレクトリパスのリストを生成"""
    if not data_dir_str:
        return []
    cleaned = data_dir_str.replace(',', ' ').replace(';', ' ').replace('"', ' ').replace("'", ' ')
    dirs = [d.strip() for d in cleaned.split() if d.strip()]
    return dirs


def generate_rviz_config(base_config_path, robot_namespace, camera_view, camera_distance):
    """namespace対応およびカメラ視点対応のRViz設定ファイルを動的に生成"""
    with open(base_config_path, 'r') as f:
        config_content = f.read()
    
    if robot_namespace:
        # TFフレーム名を置換
        config_content = config_content.replace('ref/base_link', f'{robot_namespace}/ref/base_link')
        config_content = config_content.replace('fb/base_link', f'{robot_namespace}/fb/base_link')
        config_content = config_content.replace('ref/bucket_end_link', f'{robot_namespace}/ref/bucket_end_link')
        config_content = config_content.replace('fb/bucket_end_link', f'{robot_namespace}/fb/bucket_end_link')
        
        # robot_descriptionトピック名を置換
        config_content = config_content.replace('/ref/robot_description', f'/{robot_namespace}/ref/robot_description')
        config_content = config_content.replace('/fb/robot_description', f'/{robot_namespace}/fb/robot_description')
        
        # その他のトピック名を置換
        config_content = config_content.replace('/video_gen/path_ref', f'/video_gen/{robot_namespace}/path_ref')
        config_content = config_content.replace('/video_gen/path_fb', f'/video_gen/{robot_namespace}/path_fb')
        config_content = config_content.replace('/video_gen/plan_ee_markers', f'/video_gen/{robot_namespace}/plan_ee_markers')
        config_content = config_content.replace('/video_gen/comparison_markers', f'/video_gen/{robot_namespace}/comparison_markers')
        config_content = config_content.replace('/video_gen/obstacle_markers', f'/video_gen/{robot_namespace}/obstacle_markers')
        
        # Grid的参照フレームを最初のロボットに設定
        config_content = config_content.replace('Reference Frame: ref/base_link', f'Reference Frame: {robot_namespace}/ref/base_link')
    
    # カメラ視点の切り替え（デフォルトが対角視点 3.927 になったため、diagonal指定で元の 0.785 に戻す）
    if camera_view == 'diagonal':
        config_content = config_content.replace('Yaw: 3.927', 'Yaw: 0.785')
    
    # カメラ距離の反映
    config_content = config_content.replace('Distance: 20', f'Distance: {camera_distance}')
    
    # 一時ファイルに保存
    temp_config = tempfile.NamedTemporaryFile(mode='w', suffix='.rviz', delete=False)
    temp_config.write(config_content)
    temp_config.close()
    
    return temp_config.name


def generate_nodes(context, *args, **kwargs):
    """条件に応じてノードを生成"""
    data_dir_param = LaunchConfiguration('data_dir').perform(context)
    record = LaunchConfiguration('record').perform(context)
    playback_speed = float(LaunchConfiguration('playback_speed').perform(context))
    robot_namespace = LaunchConfiguration('robot_namespace').perform(context)
    model_name = LaunchConfiguration('model_name').perform(context)
    root_record_name = LaunchConfiguration('root_record_name').perform(context)
    
    data_dirs = parse_data_dirs(data_dir_param)
    if not data_dirs:
        raise ValueError("No valid directory specified in data_dir argument.")
    
    # recordがtrueならXvfbを使用、falseなら現在のDISPLAYを使用
    display = ':99' if record.lower() == 'true' else os.environ.get('DISPLAY', ':0')
    
    # URDFとファイルパス
    urdf_file = PathJoinSubstitution([
        FindPackageShare('traj_follow_plotter'),
        'meshes',
        'zx200_video.xacro'
    ])
    
    # 動画保存先は最初の指定ディレクトリ
    output_video = os.path.join(data_dirs[0], 'animation.mp4')
    csv_paths_str = ",".join([os.path.join(d, 'data.csv') if not d.endswith('.csv') else d for d in data_dirs])
    
    # RViz設定ファイルのパス
    base_rviz_config = os.path.join(
        FindPackageShare('traj_follow_plotter').perform(context),
        'config',
        'video.rviz'
    )
    
    nodes = []
    
    # namespace用のプレフィックス（末尾にスラッシュなし、空の場合は空文字列）
    ns_prefix = robot_namespace if robot_namespace else ""
    
    # Xacroに渡すprefix（ref/, fb/の前にnamespaceを付ける）
    # 例: robot_namespace="robot1" の場合 → "robot1/ref/", "robot1/fb/"
    # 例: robot_namespace="" の場合 → "ref/", "fb/"
    if robot_namespace:
        xacro_prefix_ref = f"{robot_namespace}/ref/"
        xacro_prefix_fb = f"{robot_namespace}/fb/"
    else:
        xacro_prefix_ref = "ref/"
        xacro_prefix_fb = "fb/"
    
    # static TFで使うフレーム名（Xacroが生成するフレーム名と一致させる）
    # Xacroは prefix + "base_link" を生成するので、"robot1/ref/base_link" のようになる
    ref_base_link = f"{xacro_prefix_ref}base_link"
    fb_base_link = f"{xacro_prefix_fb}base_link"
    
    # 1. Xvfb起動（record=trueの場合のみ）
    if record.lower() == 'true':
        nodes.append(
            ExecuteProcess(
                cmd=['Xvfb', ':99', '-screen', '0', '1920x1080x24'],
                output='screen',
                name='xvfb'
            )
        )
    
    # 2. Static TF: world → {xacro_prefix}base_link
    # 2台のロボット（ref/fb）は同じ位置に重ねて表示
    x_offset = 0.0
    y_offset = 0.0
    z_offset = 0.0
    
    static_tf_ref = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'static_tf_ref_{robot_namespace}' if robot_namespace else 'static_tf_ref',
        arguments=[str(x_offset), str(y_offset), str(z_offset), '0', '0', '0', 'world', ref_base_link],
        output='screen'
    )
    nodes.append(static_tf_ref)
    
    static_tf_fb = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'static_tf_fb_{robot_namespace}' if robot_namespace else 'static_tf_fb',
        arguments=[str(x_offset), str(y_offset), str(z_offset), '0', '0', '0', 'world', fb_base_link],
        output='screen'
    )
    nodes.append(static_tf_fb)
    
    # 3. robot_state_publisher (Reference用) - 青色
    robot_state_publisher_ref = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_ref',
        namespace=f'{ns_prefix}/ref' if ns_prefix else 'ref',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file.perform(context), f' prefix:={xacro_prefix_ref}']),
        }],
        remappings=[
            ('joint_states', f'/video_gen/{ns_prefix}/joint_states_ref' if ns_prefix else '/video_gen/joint_states_ref')
        ],
        additional_env={'DISPLAY': display}
    )
    nodes.append(robot_state_publisher_ref)
    
    # 4. robot_state_publisher (Feedback用) - 緑色
    robot_state_publisher_fb = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_fb',
        namespace=f'{ns_prefix}/fb' if ns_prefix else 'fb',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file.perform(context), f' prefix:={xacro_prefix_fb}']),
        }],
        remappings=[
            ('joint_states', f'/video_gen/{ns_prefix}/joint_states_fb' if ns_prefix else '/video_gen/joint_states_fb')
        ],
        additional_env={'DISPLAY': display}
    )
    nodes.append(robot_state_publisher_fb)
    
    # 5. video_player
    video_player = Node(
        package='traj_follow_plotter',
        executable='video_player',
        name=f'video_player_node_{robot_namespace}' if robot_namespace else 'video_player_node',
        output='screen',
        parameters=[{
            'csv_path': csv_paths_str,
            'data_dir': ",".join(data_dirs),
            'playback_speed': playback_speed,
            'loop': True,
            'robot_namespace': robot_namespace,
            'model_name': model_name,
            'root_record_name': root_record_name
        }],
        remappings=[
            ('/video_gen/joint_states_ref', f'/video_gen/{ns_prefix}/joint_states_ref' if ns_prefix else '/video_gen/joint_states_ref'),
            ('/video_gen/joint_states_fb', f'/video_gen/{ns_prefix}/joint_states_fb' if ns_prefix else '/video_gen/joint_states_fb'),
            ('/video_gen/path_ref', f'/video_gen/{ns_prefix}/path_ref' if ns_prefix else '/video_gen/path_ref'),
            ('/video_gen/path_fb', f'/video_gen/{ns_prefix}/path_fb' if ns_prefix else '/video_gen/path_fb'),
            ('/video_gen/plan_ee_markers', f'/video_gen/{ns_prefix}/plan_ee_markers' if ns_prefix else '/video_gen/plan_ee_markers'),
            ('/video_gen/comparison_markers', f'/video_gen/{ns_prefix}/comparison_markers' if ns_prefix else '/video_gen/comparison_markers'),
            ('/video_gen/obstacle_markers', f'/video_gen/{ns_prefix}/obstacle_markers' if ns_prefix else '/video_gen/obstacle_markers'),
        ],
        additional_env={'DISPLAY': display}
    )
    
    video_player_delayed = TimerAction(
        period=3.0,
        actions=[video_player]
    )
    nodes.append(video_player_delayed)
    
    # 6. RViz（3秒後に起動、namespace対応のRViz設定を動的生成）
    camera_view = LaunchConfiguration('camera_view').perform(context)
    camera_distance = LaunchConfiguration('camera_distance').perform(context)
    rviz_config_path = generate_rviz_config(base_rviz_config, robot_namespace, camera_view, camera_distance)
    
    rviz = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name=f'rviz2_{robot_namespace}' if robot_namespace else 'rviz2',
                output='screen',
                arguments=['-d', rviz_config_path],
                additional_env={'DISPLAY': display}
            )
        ]
    )
    nodes.append(rviz)
    
    # 7. ffmpeg（3秒後に録画開始、record=trueの場合のみ）
    if record.lower() == 'true':
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
    
    # 8. video_playerが終了したら3秒待ってシャットダウン（record=trueの場合のみ）
    if record.lower() == 'true':
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
        description='Path to recorded data directory (or multiple directories separated by comma/space)'
    )
    
    record_arg = DeclareLaunchArgument(
        'record',
        default_value='true',
        description='Record video using Xvfb (virtual display). Set to false to use current display and check only.'
    )
    
    playback_speed_arg = DeclareLaunchArgument(
        'playback_speed',
        default_value='1.0',
        description='Playback speed multiplier (e.g., 1.0=normal, 2.0=2x speed, 0.5=half speed)'
    )
    
    robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value='',
        description='Namespace for the robot (e.g., robot1, robot2). Leave empty for no namespace.'
    )
    
    camera_view_arg = DeclareLaunchArgument(
        'camera_view',
        default_value='normal',
        description='Camera view in RViz. options: normal, diagonal'
    )
    
    camera_distance_arg = DeclareLaunchArgument(
        'camera_distance',
        default_value='20.0',
        description='Camera distance in RViz. Default is 20.0.'
    )
    
    model_name_arg = DeclareLaunchArgument(
        'model_name',
        default_value='zx200',
        description='Model name for obstacle retrieval in MongoDB.'
    )
    
    root_record_name_arg = DeclareLaunchArgument(
        'root_record_name',
        default_value='collision_objects_shimiz',
        description='Root record name for obstacle retrieval in MongoDB.'
    )
    
    return LaunchDescription([
        data_dir_arg,
        record_arg,
        playback_speed_arg,
        robot_namespace_arg,
        camera_view_arg,
        camera_distance_arg,
        model_name_arg,
        root_record_name_arg,
        OpaqueFunction(function=generate_nodes)
    ])


