#!/usr/bin/env python3
import argparse
import math
import sys
import os
import shutil

from ament_index_python.packages import get_package_share_directory
# 既存のモジュールをインポートできるようにパスを追加
# /home/common/ros2-tms-for-construction_ws/src/traj_follow_measurement/traj_follow_plotter/scripts
# の親ディレクトリの traj_follow_plotter サブモジュールを指す
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(os.path.join(parent_dir, 'traj_follow_plotter'))

from trajectory_analyzer import FKSolver, load_data_from_csv, save_data_csv

def main():
    parser = argparse.ArgumentParser(description="既存の data.csv に EE（End Effector）位置を計算して追加するスクリプト")
    parser.add_argument("input_csv", help="入力する data.csv のパス")
    
    try:
        pkg_share = get_package_share_directory('traj_follow_plotter')
        default_urdf = os.path.join(pkg_share, 'urdf', 'zx200.urdf')
    except Exception:
        default_urdf = None

    if default_urdf:
        parser.add_argument("--urdf", default=default_urdf, help=f"使用する URDF ファイルのパス (デフォルト: {default_urdf})")
    else:
        parser.add_argument("--urdf", required=True, help="使用する URDF ファイルのパス")
        
    parser.add_argument("--base-link", default="base_link", help="ベースリンク名 (デフォルト: base_link)")
    parser.add_argument("--tip-link", default="bucket_end_link", help="先端リンク(EE)名 (デフォルト: bucket_end_link)")

    args = parser.parse_args()

    input_abs = os.path.abspath(args.input_csv)
    backup_csv = os.path.join(os.path.dirname(input_abs), 'data_backup.csv')
    
    print(f"Creating backup: {args.input_csv} -> {backup_csv}")
    try:
        shutil.copy2(input_abs, backup_csv)
    except Exception as e:
        print(f"Error creating backup: {e}")
        sys.exit(1)

    output_csv = input_abs

    print(f"Loading {args.input_csv} ...")
    try:
        data = load_data_from_csv(args.input_csv)
    except Exception as e:
        print(f"Error loading CSV: {e}")
        sys.exit(1)

    t_list = data.get('t', [])
    if not t_list:
        print("Error: Input CSV contains no valid time entries ('t').")
        sys.exit(1)

    print(f"Loaded {len(t_list)} rows. Initializing FKSolver...")
    
    # FKソルバの初期化
    solver = FKSolver(args.urdf, args.base_link, args.tip_link)
    if not solver.ready:
        print("Error: Failed to initialize FKSolver. Check your URDF or link names.")
        sys.exit(1)

    fk_joints = set(solver.get_joint_names())
    csv_joints = set(data.get('joints', {}).keys())

    missing_joints = fk_joints - csv_joints
    if missing_joints:
        print(f"Warning: The following joints required for FK are not found in the CSV: {missing_joints}")
        print("         Their positions will be treated as 0.0.")

    # EEデータ格納用の辞書構造を準備
    data['ee'] = {
        'pos': {
            'ref': [[], [], []],
            'fb':  [[], [], []],
            'err': [[], [], []]
        }
    }
    data['ee_dist_err'] = []

    print("Computing EE positions...")
    for i in range(len(t_list)):
        joint_positions_ref = {}
        joint_positions_fb = {}

        # 1行ごとの関節角度を取得
        for jn in list(fk_joints):
            if jn in data['joints']:
                joint_positions_ref[jn] = data['joints'][jn]['ref'][i]
                joint_positions_fb[jn]  = data['joints'][jn]['fb'][i]
            else:
                joint_positions_ref[jn] = 0.0
                joint_positions_fb[jn]  = 0.0

        # FK計算
        ee_ref = solver.compute(joint_positions_ref)
        ee_fb  = solver.compute(joint_positions_fb)

        if ee_ref is None or ee_fb is None:
            # 計算に失敗した場合は NaN を埋める
            for axis in range(3):
                data['ee']['pos']['ref'][axis].append(math.nan)
                data['ee']['pos']['fb'][axis].append(math.nan)
                data['ee']['pos']['err'][axis].append(math.nan)
            data['ee_dist_err'].append(math.nan)
        else:
            ex = ee_fb[0] - ee_ref[0]
            ey = ee_fb[1] - ee_ref[1]
            ez = ee_fb[2] - ee_ref[2]
            dist = math.sqrt(ex*ex + ey*ey + ez*ez)

            for axis in range(3):
                data['ee']['pos']['ref'][axis].append(ee_ref[axis])
                data['ee']['pos']['fb'][axis].append(ee_fb[axis])
            
            data['ee']['pos']['err'][0].append(ex)
            data['ee']['pos']['err'][1].append(ey)
            data['ee']['pos']['err'][2].append(ez)
            
            data['ee_dist_err'].append(dist)

    print(f"Saving output to {output_csv} ...")
    save_data_csv(data, output_csv, list(data['joints'].keys()))
    print("Done!")

if __name__ == "__main__":
    main()
