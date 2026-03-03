#!/usr/bin/env python3
"""
後処理専用スクリプト
既存のCSVデータから解析・プロット・遅れ補正を実行
"""
import os
import sys
import argparse
from .trajectory_analyzer import (
    TrajectoryAnalyzer,
    load_data_from_csv,
    load_plan_from_csv,
    create_plot
)


def main():
    parser = argparse.ArgumentParser(description='Trajectory post-processing analysis')
    parser.add_argument('--dir', required=True, help='Directory containing data.csv and plan.csv')
    parser.add_argument('--output', '-o', default='plot_reanalyzed.png', help='Output PNG filename (default: plot_reanalyzed.png)')
    parser.add_argument('--max-lag', type=float, default=5.0, help='Maximum lag in seconds (default: 5.0)')
    parser.add_argument('--lag-method', choices=['correlation', 'dtw', 'frequency', 'polynomial', 'adaptive_kalman'], 
                        default='frequency', help='Lag estimation method (default: frequency)')
    
    args = parser.parse_args()
    
    # ディレクトリチェック
    if not os.path.isdir(args.dir):
        print(f"Error: Directory not found: {args.dir}")
        return 1
    
    # 入力ファイルパス
    data_csv = os.path.join(args.dir, 'data.csv')
    plan_csv = os.path.join(args.dir, 'plan.csv')
    output_png = os.path.join(args.dir, args.output)
    
    # data.csvチェック
    if not os.path.exists(data_csv):
        print(f"Error: data.csv not found in: {args.dir}")
        return 1
    
    print(f"Loading data from: {data_csv}")
    
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
    plan_data = {'t': [], 'joints': {}, 'ee': [[], [], []]}
    if os.path.exists(plan_csv):
        try:
            plan_data = load_plan_from_csv(plan_csv)
            print(f"Loaded plan data with {len(plan_data['t'])} points")
        except Exception as e:
            print(f"Warning: Failed to load plan data: {e}")
    else:
        print(f"Plan CSV not found (skipping): {plan_csv}")
    
    # 解析器作成
    analyzer = TrajectoryAnalyzer(
        max_lag_s=args.max_lag,
        lag_method=args.lag_method
    )
    
    print(f"Analyzing with method: {args.lag_method}, max_lag: {args.max_lag}s")
    
    # プロット作成
    try:
        create_plot(
            data=data,
            plan_data=plan_data,
            analyzer=analyzer,
            output_path=output_png,
            topic=os.path.basename(args.dir),
            field_label="post-process"
        )
        print(f"✓ Plot saved to: {output_png}")
        return 0
    except Exception as e:
        print(f"Error creating plot: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
