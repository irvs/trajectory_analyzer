#!/usr/bin/env python3
"""
rosbag内の/zx200/joint_statesとplan.yaml内の軌道計画をもとに、
欠損したdata.csvとplan.csvを再生成するスクリプト
/zx200/upper_arm_controller/controller_stateがrosbagから抜けている際に利用する
"""
import os
import sys
import argparse
import yaml
import csv
import math
from typing import List, Dict, Optional, Tuple
import bisect
import numpy as np

try:
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    import rclpy
except ImportError as e:
    print(f"Error: ROS2 Python environment not found: {e}")
    sys.exit(1)

# 自パッケージのモジュールをインポートするためにパスを追加
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.append(script_dir)

from trajectory_analyzer import FKSolver, save_data_csv, save_plan_csv, compute_plan_ee_positions

def get_default_urdf_path():
    """デフォルトのURDFパスを取得"""
    try:
        from ament_index_python.packages import get_package_share_directory
        pkg_share = get_package_share_directory('traj_follow_plotter')
        return os.path.join(pkg_share, "urdf", "zx200.urdf")
    except Exception:
        return "install/traj_follow_plotter/share/traj_follow_plotter/urdf/zx200.urdf"

def load_plan_yaml(yaml_path: str) -> dict:
    """plan.yamlからデータを読み込む"""
    with open(yaml_path, 'r') as f:
        plan_data = yaml.safe_load(f)
    
    joint_traj = plan_data['plan']['joint_trajectory']
    joint_names = joint_traj['joint_names']
    
    # データを整形
    t_points = []
    positions = []
    
    for pt in joint_traj['points']:
        t_sec = pt['time_from_start']['sec'] + pt['time_from_start']['nanosec'] * 1e-9
        t_points.append(t_sec)
        positions.append(pt['positions'])
        
    return {
        'joint_names': joint_names,
        't': t_points,
        'positions': positions
    }

def extract_joint_states_from_bag(bag_dir: str, topic: str = "/zx200/joint_states"):
    """Bagからjoint_statesを抽出"""
    storage_options = StorageOptions(uri=bag_dir, storage_id='sqlite3')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    
    joint_states = []
    start_time = None
    
    msg_type = get_message('sensor_msgs/msg/JointState')
    
    while reader.has_next():
        (t_name, data, timestamp) = reader.read_next()
        if t_name == topic:
            msg = deserialize_message(data, msg_type)
            if start_time is None:
                start_time = timestamp
            
            t_rel = (timestamp - start_time) * 1e-9
            joint_states.append((t_rel, msg))
            
    return joint_states

def main():
    parser = argparse.ArgumentParser(description='Regenerate data.csv from bag and plan.yaml')
    parser.add_argument('run_dir', type=str, help='Path to run_* directory')
    parser.add_argument('--urdf', type=str, default=None, help='Path to URDF file')
    args = parser.parse_args()
    
    run_dir = os.path.abspath(args.run_dir)
    plan_yaml_path = os.path.join(run_dir, 'plan.yaml')
    bag_dir = os.path.join(run_dir, 'bag')
    
    if not os.path.exists(plan_yaml_path):
        print(f"Error: plan.yaml not found at {plan_yaml_path}")
        return
    if not os.path.exists(bag_dir):
        print(f"Error: bag directory not found at {bag_dir}")
        return

    # 1. Plan読み込み
    print(f"Loading plan from {plan_yaml_path}...")
    plan = load_plan_yaml(plan_yaml_path)
    
    # 2. Bag読み込み
    print(f"Loading joint_states from {bag_dir}...")
    actual_states = extract_joint_states_from_bag(bag_dir)
    print(f"Extracted {len(actual_states)} joint state samples.")
    
    if not actual_states:
        print("Error: No joint state samples found.")
        return

    # 3. URDF/FK準備
    urdf_path = args.urdf if args.urdf else get_default_urdf_path()
    fk_solver = FKSolver(urdf_path, base_link="base_link", tip_link="bucket_end_link")
    chain_joint_names = fk_solver.get_joint_names() if fk_solver.ready else []

    # 4. データ合成開始 (10Hz にリサンプリング: joint_states の発行レートに合わせる)
    print("Resampling and interpolating data at 10Hz...")
    freq = 10.0
    dt_step = 1.0 / freq
    total_duration = actual_states[-1][0]
    n_samples = int(total_duration * freq) + 1

    out_data = {
        't': [],
        'joints': {}
    }

    # data.csvに含める関節名
    target_joint_names = [jn for jn in plan['joint_names'] if jn != 'bucket_end_joint']
    for jn in target_joint_names:
        out_data['joints'][jn] = {'ref': [], 'fb': [], 'err': [], 'vel': []}

    if fk_solver.ready:
        out_data['ee'] = {
            'pos': {
                'ref': [[], [], []],
                'fb':  [[], [], []],
                'err': [[], [], []]
            }
        }
        out_data['ee_dist_err'] = []

    # ── Actual 側: 補間用データを事前に抽出 ──
    actual_t = [s[0] for s in actual_states]
    actual_joint_map = {}  # {jn: [pos, ...]}
    actual_vel_map = {}    # {jn: [vel, ...]}

    msg_joint_names = list(actual_states[0][1].name)
    for jn in target_joint_names:
        try:
            a_idx = msg_joint_names.index(jn)
            actual_joint_map[jn] = [s[1].position[a_idx] for s in actual_states]
            actual_vel_map[jn] = [s[1].velocity[a_idx] if s[1].velocity else 0.0 for s in actual_states]
        except (ValueError, IndexError):
            actual_joint_map[jn] = [0.0] * len(actual_states)
            actual_vel_map[jn] = [0.0] * len(actual_states)

    # ── Plan 側: 補間用データを事前に抽出 (errorを連続値にするため線形補間を適用) ──
    plan_joint_map = {}  # {jn: [pos, ...]} (plan の生データ)
    for jn in target_joint_names:
        try:
            p_idx = plan['joint_names'].index(jn)
            plan_joint_map[jn] = [pos[p_idx] for pos in plan['positions']]
        except (ValueError, IndexError):
            plan_joint_map[jn] = [0.0] * len(plan['t'])

    # 補間実行 (10Hz グリッド)
    for i in range(n_samples):
        t_rel = i * dt_step
        out_data['t'].append(t_rel)

        jp_ref_map = {}
        jp_fb_map = {}

        for jn in target_joint_names:
            # Plan側: 線形補間 (error の離散化を防ぐ)
            p_val = float(np.interp(t_rel, plan['t'], plan_joint_map[jn]))

            # Actual側: 線形補間
            a_val = float(np.interp(t_rel, actual_t, actual_joint_map[jn]))
            v_val = float(np.interp(t_rel, actual_t, actual_vel_map[jn]))

            out_data['joints'][jn]['ref'].append(p_val)
            out_data['joints'][jn]['fb'].append(a_val)
            out_data['joints'][jn]['err'].append(a_val - p_val)
            out_data['joints'][jn]['vel'].append(v_val)

            jp_ref_map[jn] = p_val
            jp_fb_map[jn] = a_val

        # EE計算
        if fk_solver.ready:
            ee_ref = fk_solver.compute(jp_ref_map)
            ee_fb = fk_solver.compute(jp_fb_map)
            
            if ee_ref and ee_fb:
                dist_err = math.sqrt(sum((ee_fb[i] - ee_ref[i])**2 for i in range(3)))
                out_data['ee_dist_err'].append(dist_err)
                for i in range(3):
                    out_data['ee']['pos']['ref'][i].append(ee_ref[i])
                    out_data['ee']['pos']['fb'][i].append(ee_fb[i])
                    out_data['ee']['pos']['err'][i].append(ee_fb[i] - ee_ref[i])
            else:
                out_data['ee_dist_err'].append(0.0)
                for i in range(3):
                    out_data['ee']['pos']['ref'][i].append(0.0)
                    out_data['ee']['pos']['fb'][i].append(0.0)
                    out_data['ee']['pos']['err'][i].append(0.0)

    # 5. 保存
    data_csv_path = os.path.join(run_dir, 'data.csv')
    save_data_csv(out_data, data_csv_path, target_joint_names)
    print(f"✓ Created {data_csv_path}")
    
    # 6. plan.csv も作成
    plan_csv_path = os.path.join(run_dir, 'plan.csv')
    # plan データ辞書作成
    plan_dict = {
        't': plan['t'],
        'joints': {}
    }
    for jn in target_joint_names:
        p_idx = plan['joint_names'].index(jn)
        plan_dict['joints'][jn] = [pos[p_idx] for pos in plan['positions']]
    
    # EE for plan
    ee_plan = compute_plan_ee_positions(plan_dict, urdf_path)
    if ee_plan:
        plan_dict['ee'] = [[], [], []]
        for x, y, z in ee_plan:
            plan_dict['ee'][0].append(x)
            plan_dict['ee'][1].append(y)
            plan_dict['ee'][2].append(z)
            
    save_plan_csv(plan_dict, plan_csv_path, target_joint_names)
    print(f"✓ Created {plan_csv_path}")
    
    print("\nGeneration complete. You can now run video generation or trajectory analysis.")

if __name__ == "__main__":
    main()
