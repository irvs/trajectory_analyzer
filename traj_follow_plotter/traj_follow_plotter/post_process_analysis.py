#!/usr/bin/env python3
"""
後処理専用スクリプト
rosbagファイルからlink_padding解析を実行
"""
import os
import sys
import argparse
import yaml


def get_default_urdf_path():
    """デフォルトのURDFパスを取得"""
    try:
        from ament_index_python.packages import get_package_share_directory
        pkg_share = get_package_share_directory('traj_follow_plotter')
        return os.path.join(pkg_share, "urdf", "zx200.urdf")
    except Exception:
        # パッケージが見つからない場合は相対パスを返す
        return "install/traj_follow_plotter/share/traj_follow_plotter/urdf/zx200.urdf"


def extract_data_from_bag(bag_path: str, output_dir: str, urdf_path: str, state_topic: str = None) -> tuple:
    """
    rosbagファイルからdata.csvとplan.csvを抽出
    
    Returns:
        (data_csv_path, plan_csv_path) or (None, None) on error
    """
    print(f"Extracting data from bag: {bag_path}")
    
    try:
        from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
        import rclpy
        
        # rclpy初期化（必要な場合）
        if not rclpy.ok():
            rclpy.init()
        
    except ImportError as e:
        print(f"Error: Required ROS2 packages not found: {e}")
        print("Make sure rosbag2_py is installed: sudo apt install ros-humble-rosbag2-py")
        return None, None
    
    # Bagファイルを開く
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    
    # トピック情報を取得
    topic_types = reader.get_all_topics_and_types()
    
    # controller_state トピックを探す
    state_topic_found = None
    plan_topic_found = None
    
    for topic_metadata in topic_types:
        topic_name = topic_metadata.name
        topic_type = topic_metadata.type
        
        # controller_state トピック（JointTrajectoryControllerState）
        if 'controller_state' in topic_name and 'JointTrajectoryControllerState' in topic_type:
            if state_topic is None or state_topic == topic_name:
                state_topic_found = topic_name
                print(f"Found controller state topic: {state_topic_found}")
        
        # plan トピック（action goal）
        if 'traj_follow_record' in topic_name and '_action/goal' in topic_name:
            plan_topic_found = topic_name
            print(f"Found plan topic: {plan_topic_found}")
    
    if not state_topic_found:
        print("Error: No JointTrajectoryControllerState topic found in bag")
        return None, None
    
    # データ収集
    state_data = []
    plan_data = None
    start_time = None
    
    print("Reading bag file...")
    while reader.has_next():
        (topic, data, timestamp) = reader.read_next()
        
        if topic == state_topic_found:
            # controller_state メッセージをデシリアライズ
            msg_type = get_message('control_msgs/msg/JointTrajectoryControllerState')
            msg = deserialize_message(data, msg_type)
            
            if start_time is None:
                start_time = timestamp
            
            t_rel = (timestamp - start_time) * 1e-9  # nanosec to sec
            state_data.append((t_rel, msg))
        
        elif topic == plan_topic_found and plan_data is None:
            # Action goal メッセージをデシリアライズ
            msg_type = get_message('traj_recorder_msgs/action/TrajFollow_Goal')
            msg = deserialize_message(data, msg_type)
            plan_data = msg
    
    print(f"Collected {len(state_data)} state samples")
    
    if len(state_data) == 0:
        print("Error: No state data found in bag")
        return None, None
    
    # ===== 共通モジュールを使ってCSVに変換 =====
    from .trajectory_analyzer import save_data_csv, save_plan_csv, compute_plan_ee_positions
    
    # data.csv を生成
    data_csv_path = os.path.join(output_dir, 'data.csv')
    data_dict = _convert_state_data_to_dict(state_data)
    
    # 関節名リスト（bucket_end_jointを除外）
    joint_names = [jn for jn in data_dict['joints'].keys() if jn != 'bucket_end_joint']
    
    save_data_csv(data_dict, data_csv_path, joint_names)
    print(f"Saved data.csv: {data_csv_path}")
    
    # ===== plan.yamlを親ディレクトリから探す =====
    # bag_path = .../run_XXX/bag/ なので、親ディレクトリは .../run_XXX/
    bag_parent_dir = os.path.dirname(os.path.abspath(bag_path))
    plan_yaml_path = os.path.join(bag_parent_dir, 'plan.yaml')
    
    plan_csv_path = None
    if os.path.exists(plan_yaml_path):
        print(f"Found plan.yaml in parent directory: {plan_yaml_path}")
        plan_csv_path = os.path.join(output_dir, 'plan.csv')
        
        # plan.yamlを読み込んでplan.csvに変換
        plan_dict = _load_plan_from_yaml(plan_yaml_path, urdf_path)
        
        if plan_dict and plan_dict['t']:
            save_plan_csv(plan_dict, plan_csv_path, joint_names)
            print(f"Saved plan.csv: {plan_csv_path}")
        else:
            print("Warning: Failed to load plan data from plan.yaml")
            plan_csv_path = None
    else:
        print(f"Warning: plan.yaml not found in parent directory: {bag_parent_dir}")
        print("  Plan data will not be available for analysis.")
    
    return data_csv_path, plan_csv_path


def _convert_state_data_to_dict(state_data: list) -> dict:
    """controller_state データを辞書形式に変換（共通モジュール用）"""
    # 関節名を取得（最初のメッセージから）
    _, first_msg = state_data[0]
    
    # reference/feedback または desired/actual を判定
    if hasattr(first_msg, 'reference'):
        ref_field = 'reference'
        fb_field = 'feedback'
        err_field = 'error'
    else:
        ref_field = 'desired'
        fb_field = 'actual'
        err_field = 'error'
    
    joint_names = list(first_msg.joint_names) if hasattr(first_msg, 'joint_names') else []
    
    # bucket_end_joint は除外
    joint_names = [jn for jn in joint_names if jn != 'bucket_end_joint']
    
    # 辞書を初期化
    data = {
        't': [],
        'joints': {}
    }
    
    for jn in joint_names:
        data['joints'][jn] = {
            'ref': [],
            'fb': [],
            'err': [],
            'vel': []
        }
    
    # データを格納
    for t_rel, msg in state_data:
        ref_pt = getattr(msg, ref_field)
        fb_pt = getattr(msg, fb_field)
        err_pt = getattr(msg, err_field)
        
        ref_pos = list(ref_pt.positions) if hasattr(ref_pt, 'positions') else []
        fb_pos = list(fb_pt.positions) if hasattr(fb_pt, 'positions') else []
        err_pos = list(err_pt.positions) if hasattr(err_pt, 'positions') else []
        fb_vel = list(fb_pt.velocities) if hasattr(fb_pt, 'velocities') else []
        
        data['t'].append(t_rel)
        
        for jn in joint_names:
            # joint_namesからインデックスを取得
            if hasattr(msg, 'joint_names'):
                try:
                    idx = list(msg.joint_names).index(jn)
                except ValueError:
                    idx = joint_names.index(jn)
            else:
                idx = joint_names.index(jn)
            
            data['joints'][jn]['ref'].append(ref_pos[idx] if idx < len(ref_pos) else 0.0)
            data['joints'][jn]['fb'].append(fb_pos[idx] if idx < len(fb_pos) else 0.0)
            data['joints'][jn]['err'].append(err_pos[idx] if idx < len(err_pos) else 0.0)
            data['joints'][jn]['vel'].append(fb_vel[idx] if idx < len(fb_vel) else 0.0)
    
    return data


def _convert_plan_data_to_dict(plan_goal_msg, urdf_path: str) -> dict:
    """Plan データを辞書形式に変換（共通モジュール用、EE位置も計算）"""
    from .trajectory_analyzer import compute_plan_ee_positions
    
    # Planから軌道を取得
    joint_traj = plan_goal_msg.plan.joint_trajectory
    joint_names = list(joint_traj.joint_names)
    
    # bucket_end_joint は除外
    joint_names_filtered = [jn for jn in joint_names if jn != 'bucket_end_joint']
    
    plan_dict = {
        't': [],
        'joints': {}
    }
    
    for jn in joint_names_filtered:
        plan_dict['joints'][jn] = []
    
    for point in joint_traj.points:
        t_sec = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
        positions = list(point.positions)
        
        plan_dict['t'].append(t_sec)
        
        for jn in joint_names_filtered:
            idx = joint_names.index(jn)
            plan_dict['joints'][jn].append(positions[idx] if idx < len(positions) else 0.0)
    
    # EE位置を計算
    ee_positions = compute_plan_ee_positions(plan_dict, urdf_path)
    
    if ee_positions:
        plan_dict['ee'] = [[], [], []]
        for x, y, z in ee_positions:
            plan_dict['ee'][0].append(x)
            plan_dict['ee'][1].append(y)
            plan_dict['ee'][2].append(z)
    
    return plan_dict


def _load_plan_from_yaml(yaml_path: str, urdf_path: str) -> dict:
    """plan.yamlからPlanデータを読み込んで辞書形式に変換"""
    try:
        with open(yaml_path, 'r') as f:
            plan_yaml = yaml.safe_load(f)
        
        # plan.joint_trajectory を取得
        joint_traj = plan_yaml['plan']['joint_trajectory']
        joint_names = joint_traj['joint_names']
        
        # bucket_end_joint は除外
        joint_names_filtered = [jn for jn in joint_names if jn != 'bucket_end_joint']
        
        plan_dict = {
            't': [],
            'joints': {}
        }
        
        for jn in joint_names_filtered:
            plan_dict['joints'][jn] = []
        
        # 各pointから時刻と位置を抽出
        for point in joint_traj['points']:
            # time_from_startを秒に変換
            t_sec = point['time_from_start']['sec'] + point['time_from_start']['nanosec'] * 1e-9
            positions = point['positions']
            
            plan_dict['t'].append(t_sec)
            
            for jn in joint_names_filtered:
                idx = joint_names.index(jn)
                plan_dict['joints'][jn].append(positions[idx] if idx < len(positions) else 0.0)
        
        # EE位置を計算
        from .trajectory_analyzer import compute_plan_ee_positions
        ee_positions = compute_plan_ee_positions(plan_dict, urdf_path)
        
        if ee_positions:
            plan_dict['ee'] = [[], [], []]
            for x, y, z in ee_positions:
                plan_dict['ee'][0].append(x)
                plan_dict['ee'][1].append(y)
                plan_dict['ee'][2].append(z)
        
        print(f"Loaded plan from YAML: {len(plan_dict['t'])} points")
        return plan_dict
        
    except Exception as e:
        print(f"Error loading plan from YAML: {e}")
        import traceback
        traceback.print_exc()
        return None


def main():
    parser = argparse.ArgumentParser(
        description='Link padding post-processing analysis from rosbag',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Using default URDF:
  %(prog)s --bag /path/to/rosbag_dir
  
  # With custom URDF:
  %(prog)s --bag /path/to/rosbag_dir --urdf /path/to/robot.urdf
        """
    )
    
    # 必須引数
    parser.add_argument('--bag', type=str, required=True,
                        help='Path to rosbag2 directory')
    
    # オプション引数（URDFもデフォルト値を持つ）
    parser.add_argument('--urdf', type=str, default=None,
                        help='Path to URDF file (default: package zx200.urdf)')
    parser.add_argument('--base-link', type=str, default='base_link',
                        help='Base link name for FK (default: base_link)')
    parser.add_argument('--output-dir', type=str, default=None,
                        help='Output directory (default: bag_parent/analysis_output)')
    parser.add_argument('--state-topic', type=str, default=None,
                        help='Controller state topic name (auto-detect if not specified)')
    
    args = parser.parse_args()
    
    # URDFパスの決定（指定なしの場合はデフォルト値）
    urdf_path = args.urdf if args.urdf else get_default_urdf_path()
    
    # URDFチェック
    if not os.path.exists(urdf_path):
        print(f"Error: URDF not found: {urdf_path}")
        print(f"Please specify URDF path with --urdf option")
        return 1
    
    print(f"Using URDF: {urdf_path}")
    
    # Bagファイルチェック
    if not os.path.exists(args.bag):
        print(f"Error: Bag file not found: {args.bag}")
        return 1
    
    # 出力ディレクトリ決定
    if args.output_dir:
        output_dir = args.output_dir
    else:
        # bagファイルの親ディレクトリに analysis_output を作成
        bag_parent = os.path.dirname(os.path.abspath(args.bag))
        output_dir = os.path.join(bag_parent, 'analysis_output')
    
    os.makedirs(output_dir, exist_ok=True)
    print(f"Output directory: {output_dir}")
    
    # bagから抽出
    data_csv, plan_csv = extract_data_from_bag(args.bag, output_dir, urdf_path, args.state_topic)
    
    if data_csv is None:
        return 1
    
    # ===== link_padding解析を実行 =====
    print(f"\nLoading data from: {data_csv}")
    
    from .trajectory_analyzer import (
        TrajectoryAnalyzer,
        load_data_from_csv,
        load_plan_from_csv,
        analyze_link_padding
    )
    
    # データ読み込み
    try:
        data = load_data_from_csv(data_csv)
        print(f"Loaded {len(data['t'])} samples")
        print(f"Joints: {list(data['joints'].keys())}")
    except Exception as e:
        print(f"Error loading data: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    # Planデータ読み込み
    if plan_csv and os.path.exists(plan_csv):
        try:
            plan_data = load_plan_from_csv(plan_csv)
            print(f"Loaded plan data with {len(plan_data['t'])} points")
        except Exception as e:
            print(f"Error loading plan data: {e}")
            import traceback
            traceback.print_exc()
            return 1
    else:
        print("Error: plan.csv not found")
        return 1
    
    # 解析器作成
    analyzer = TrajectoryAnalyzer()
    
    # Link padding解析を実行
    try:
        print("\nPerforming link padding analysis...")
        analyze_link_padding(
            data=data,
            plan_data=plan_data,
            analyzer=analyzer,
            urdf_path=urdf_path,  # ★デフォルトまたは指定されたURDF
            output_dir=output_dir,
            base_link=args.base_link
        )
        
        print(f"\n✓ Analysis complete! Check {output_dir} for results:")
        print(f"  - data.csv                          (Extracted state data)")
        print(f"  - plan.csv                          (Extracted plan data)")
        print(f"  - link_padding_summary.png          (Main visualization)")
        print(f"  - link_padding_summary.yaml         (Statistics)")
        print(f"  - link_padding_summary.csv          (Statistics CSV)")
        print(f"  - link_correspondence_nearest.csv   (Correspondence data for video)")
        print(f"  - link_padding_report.txt           (Human-readable report)")
        
        return 0
    except Exception as e:
        print(f"Error in link padding analysis: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
