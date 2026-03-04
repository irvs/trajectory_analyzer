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
    create_plot,
    save_compensated_csv
)


def main():
    parser = argparse.ArgumentParser(description='Trajectory post-processing analysis')
    parser.add_argument('--dir', required=True, help='Directory containing data.csv and plan.csv')
    parser.add_argument('--output', '-o', default='plot_reanalyzed.png', help='Output PNG filename (default: plot_reanalyzed.png)')
    parser.add_argument('--max-lag', type=float, default=5.0, help='Maximum lag in seconds (default: 5.0)')
    parser.add_argument('--lag-method', choices=['correlation', 'dtw', 'frequency', 'polynomial', 'adaptive_kalman', 'progress'], 
                        default='progress', help='Lag estimation method (default: progress)')
    parser.add_argument('--no-save-compensated', action='store_true', 
                        help='Do NOT save compensated feedback data (by default it is saved)')
    parser.add_argument('--urdf', type=str, default=None,
                        help='Path to URDF file for FK-based EE position compensation')
    parser.add_argument('--base-link', type=str, default='base_link',
                        help='Base link name for FK (default: base_link)')
    parser.add_argument('--tip-link', type=str, default='bucket_end_link',
                        help='Tip link name for FK (default: bucket_end_link)')
    
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
    
    # 補正済みCSVを保存（デフォルトで保存、--no-save-compensatedで無効化）
    if not args.no_save_compensated:
        compensated_csv = os.path.join(args.dir, 'data_compensated.csv')
        try:
            save_compensated_csv(
                data, 
                analyzer, 
                compensated_csv,
                urdf_path=args.urdf,
                base_link=args.base_link,
                tip_link=args.tip_link
            )
            print(f"✓ Compensated data saved to: {compensated_csv}")
        except Exception as e:
            print(f"Error saving compensated CSV: {e}")
            import traceback
            traceback.print_exc()
    
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
