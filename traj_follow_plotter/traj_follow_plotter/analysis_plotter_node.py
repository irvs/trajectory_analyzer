#!/usr/bin/env python3
import os
import sys
import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt

# 共通モジュールをインポート
from .trajectory_analyzer import (
    TrajectoryAnalyzer,
    load_data_from_csv,
    load_plan_from_csv,
    analyze_link_padding,
    save_data_csv,
    save_plan_csv
)

def get_default_urdf_path():
    """デフォルトのURDFパスを取得"""
    try:
        from ament_index_python.packages import get_package_share_directory
        pkg_share = get_package_share_directory('traj_follow_plotter')
        return os.path.normpath(os.path.join(pkg_share, "urdf", "zx200.urdf"))
    except Exception:
        return "install/traj_follow_plotter/share/traj_follow_plotter/urdf/zx200.urdf"

class AnalysisPlotterNode(Node):
    """data.csv / plan.csv を解析して画像を生成するノード"""
    
    def __init__(self):
        super().__init__("analysis_plotter")
        
        # パラメータ定義
        self.declare_parameter("data_dir", "")
        self.declare_parameter("output_dir", "")
        self.declare_parameter("urdf_path", get_default_urdf_path())
        self.declare_parameter("base_link", "base_link")
        self.declare_parameter("tip_link", "bucket_end_link")
        self.declare_parameter("time_limit", 30.0)
        
        data_dir_param = str(self.get_parameter("data_dir").value)
        output_dir_param = str(self.get_parameter("output_dir").value)
        urdf_path = str(self.get_parameter("urdf_path").value)
        base_link = str(self.get_parameter("base_link").value)
        tip_link = str(self.get_parameter("tip_link").value)
        time_limit = float(self.get_parameter("time_limit").value)
        
        # 対象のディレクトリリストを解析
        dir_list = self._parse_dir_list(data_dir_param)
        if not dir_list:
            self.get_logger().error("No valid data directories specified via 'data_dir' parameter.")
            sys.exit(1)
            
        # 出力先の決定
        output_dir = output_dir_param if output_dir_param else dir_list[0]
        os.makedirs(output_dir, exist_ok=True)
        
        self.get_logger().info(f"Target directories: {dir_list}")
        self.get_logger().info(f"Output directory: {output_dir}")
        self.get_logger().info(f"Using URDF: {urdf_path}")
        
        # URDFの存在確認
        if not os.path.exists(urdf_path):
            self.get_logger().error(f"URDF path does not exist: {urdf_path}")
            sys.exit(1)
            
        # 1. データのロードと連結
        self.get_logger().info("Loading and concatenating data...")
        concated_data = self._load_and_concat_data(dir_list)
        concated_plan = self._load_and_concat_plan(dir_list)
        
        if not concated_data or not concated_data['t']:
            self.get_logger().error("No valid trajectory data could be loaded.")
            sys.exit(1)

        # 指定した秒数までのデータを切り出す
        if concated_data['t'][-1] > time_limit:
            concated_data = self._crop_data_by_time(concated_data, time_limit)
            concated_plan = self._crop_data_by_time(concated_plan, time_limit)
        
        # -PI~PIの範囲になるように調整
        for i in range(len(concated_data['t'])):
            for jn in concated_data['joints']:
                concated_data['joints'][jn]['ref'][i] = np.arctan2(np.sin(concated_data['joints'][jn]['ref'][i]), np.cos(concated_data['joints'][jn]['ref'][i]))
                concated_data['joints'][jn]['fb'][i] = np.arctan2(np.sin(concated_data['joints'][jn]['fb'][i]), np.cos(concated_data['joints'][jn]['fb'][i]))
                concated_data['joints'][jn]['err'][i] = np.arctan2(np.sin(concated_data['joints'][jn]['err'][i]), np.cos(concated_data['joints'][jn]['err'][i]))
            
        # 連結したCSVファイルを保存する
        self.get_logger().info("Saving concatenated CSV files to output directory...")
        joint_names = ['swing_joint', 'boom_joint', 'arm_joint', 'bucket_joint']
        save_data_csv(concated_data, os.path.join(output_dir, "data_concatenated.csv"), joint_names)
        if concated_plan and concated_plan['t']:
            save_plan_csv(concated_plan, os.path.join(output_dir, "plan_concatenated.csv"), joint_names)
            
        # 2. 既存のLink Padding解析の実行
        self.get_logger().info("Running link padding analysis...")
        analyzer = TrajectoryAnalyzer()
        try:
            analyze_link_padding(
                data=concated_data,
                plan_data=concated_plan,
                analyzer=analyzer,
                urdf_path=urdf_path,
                output_dir=output_dir,
                base_link=base_link
            )
            self.get_logger().info("✓ Link padding analysis complete.")
        except Exception as e:
            self.get_logger().error(f"Failed standard link padding analysis: {e}")
            
        # 3. 新規の目標角度と実機角度の時間推移プロット
        self.get_logger().info("Generating joint angle tracking plot...")
        self._plot_joint_angles(concated_data, joint_names, output_dir)
        
        self.get_logger().info("✓ Processing complete. Exiting...")
    
    def _crop_data_by_time(self, data, time_limit):
        """指定した秒数までのデータを切り出す"""
        if not data['t']:
            return data
        
        for i in range(len(data['t'])):
            if data['t'][i] > time_limit:
                data['t'] = data['t'][:i]
                for jn in data['joints']:
                    data['joints'][jn]['ref'] = data['joints'][jn]['ref'][:i]
                    data['joints'][jn]['fb'] = data['joints'][jn]['fb'][:i]
                    data['joints'][jn]['err'] = data['joints'][jn]['err'][:i]
                    if 'vel' in data['joints'][jn]:
                        data['joints'][jn]['vel'] = data['joints'][jn]['vel'][:i]
                break
        return data
        
    def _parse_dir_list(self, data_dir_param: str):
        """パラメータからディレクトリリストをパース"""
        if not data_dir_param:
            return []
        cleaned = data_dir_param.replace(',', ' ').replace(';', ' ')
        tokens = [t.strip() for t in cleaned.split() if t.strip()]
        
        dir_list = []
        for token in tokens:
            if os.path.isfile(token):
                dir_path = os.path.dirname(token)
            else:
                dir_path = token
            if dir_path and dir_path not in dir_list:
                dir_list.append(dir_path)
        return dir_list

    def _load_and_concat_data(self, dir_list):
        """複数のdata.csvを読み込んで連結（時間軸調整）"""
        concated_data = {
            't': [],
            'joints': {},
            'ee': {},
            'ee_dist_err': []
        }
        t_offset = 0.0
        
        for run_dir in dir_list:
            csv_path = os.path.join(run_dir, "data.csv") if not run_dir.endswith(".csv") else run_dir
            if not os.path.exists(csv_path):
                self.get_logger().warn(f"data.csv not found in: {run_dir}")
                continue
                
            data = load_data_from_csv(csv_path)
            if not data or not data.get('t'):
                self.get_logger().warn(f"No valid data in: {csv_path}")
                continue
                
            # 時間軸シフト
            t_shifted = [t + t_offset for t in data['t']]
            concated_data['t'].extend(t_shifted)
            
            # 関節データマージ
            for jn, jdata in data['joints'].items():
                if jn not in concated_data['joints']:
                    concated_data['joints'][jn] = {'ref': [], 'fb': [], 'err': [], 'vel': []}
                concated_data['joints'][jn]['ref'].extend(jdata['ref'])
                concated_data['joints'][jn]['fb'].extend(jdata['fb'])
                concated_data['joints'][jn]['err'].extend(jdata['err'])
                if 'vel' in jdata:
                    concated_data['joints'][jn]['vel'].extend(jdata['vel'])
                    
            # EEデータマージ
            if 'pos' in data.get('ee', {}):
                if 'pos' not in concated_data['ee']:
                    concated_data['ee']['pos'] = {'ref': [[], [], []], 'fb': [[], [], []], 'err': [[], [], []]}
                for axis in range(3):
                    concated_data['ee']['pos']['ref'][axis].extend(data['ee']['pos']['ref'][axis])
                    concated_data['ee']['pos']['fb'][axis].extend(data['ee']['pos']['fb'][axis])
                    concated_data['ee']['pos']['err'][axis].extend(data['ee']['pos']['err'][axis])
                    
            # 距離誤差マージ
            if data.get('ee_dist_err'):
                concated_data['ee_dist_err'].extend(data['ee_dist_err'])
            elif data.get('t'):
                # 存在しない場合はダミーで埋める
                concated_data['ee_dist_err'].extend([0.0] * len(data['t']))
                
            # 次のファイル用のタイムオフセット更新
            dt = 0.033
            if len(data['t']) >= 2:
                dt = data['t'][1] - data['t'][0]
            t_offset = t_shifted[-1] + dt
            
        return concated_data

    def _load_and_concat_plan(self, dir_list):
        """複数のplan.csvを読み込んで連結（時間軸調整）"""
        concated_plan = {
            't': [],
            'joints': {},
            'ee': [[], [], []]
        }
        t_offset = 0.0
        
        for run_dir in dir_list:
            csv_path = os.path.join(run_dir, "plan.csv")
            # plan.csvが存在しない場合はplan.yamlを探して変換
            if not os.path.exists(csv_path):
                # 親ディレクトリや同じ場所にplan.yamlがあるか探す
                plan_yaml_path = os.path.join(run_dir, "plan.yaml")
                if os.path.exists(plan_yaml_path):
                    self.get_logger().info(f"plan.csv not found, but plan.yaml exists. Generating plan.csv for: {run_dir}")
                    # post_process_analysis の plan ロード処理を流用
                    try:
                        from .post_process_analysis import _load_plan_from_yaml
                        urdf_path = str(self.get_parameter("urdf_path").value)
                        plan_dict = _load_plan_from_yaml(plan_yaml_path, urdf_path)
                        if plan_dict and plan_dict['t']:
                            joint_names = ['swing_joint', 'boom_joint', 'arm_joint', 'bucket_joint']
                            save_plan_csv(plan_dict, csv_path, joint_names)
                    except Exception as e:
                        self.get_logger().warn(f"Failed to generate plan.csv from plan.yaml: {e}")
            
            if not os.path.exists(csv_path):
                self.get_logger().warn(f"plan.csv not found in: {run_dir}")
                continue
                
            plan = load_plan_from_csv(csv_path)
            if not plan or not plan.get('t'):
                self.get_logger().warn(f"No valid plan data in: {csv_path}")
                continue
                
            # 時間軸シフト
            t_shifted = [t + t_offset for t in plan['t']]
            concated_plan['t'].extend(t_shifted)
            
            # 関節データマージ
            for jn, jpos in plan['joints'].items():
                if jn not in concated_plan['joints']:
                    concated_plan['joints'][jn] = []
                concated_plan['joints'][jn].extend(jpos)
                
            # EEデータマージ
            if plan.get('ee') and len(plan['ee']) == 3:
                for axis in range(3):
                    concated_plan['ee'][axis].extend(plan['ee'][axis])
                    
            # タイムオフセット更新
            dt = 0.1
            if len(plan['t']) >= 2:
                dt = plan['t'][1] - plan['t'][0]
            t_offset = t_shifted[-1] + dt
            
        return concated_plan

    def _plot_joint_angles(self, data, joint_names, output_dir):
        """目標角度と実機角度の時間推移をプロット"""
        fig, axs = plt.subplots(4, 1, sharex=False, figsize=(12, 10))
        t = data['t']
        
        # 美しい見た目のためのスタイル設定
        plt.rcParams['font.sans-serif'] = 'DejaVu Sans'
        plt.rcParams['font.family'] = 'sans-serif'
        
        # 各関節ごとの描画色
        ref_color = '#1f77b4' # 青
        fb_color = '#2ca02c'  # 緑
        
        for i, jn in enumerate(joint_names):
            ax = axs[i]
            
            if jn in data['joints']:
                ref = data['joints'][jn]['ref']
                fb = data['joints'][jn]['fb']
                
                # プロット
                ax.plot(t, ref, color=ref_color, linestyle='-', linewidth=1.5, label='Target (Reference)')
                ax.plot(t, fb, color=fb_color, linestyle='--', linewidth=1.5, label='Actual (Feedback)')
                
                # タイトルとラベル設定
                ax.set_ylabel('Angle [rad]', fontsize=10, fontweight='bold')
                ax.set_title(f'{jn}', fontsize=12, fontweight='bold', loc='left')
                ax.grid(True, linestyle=':', alpha=0.6)
                # ax.legend(loc='upper right', framealpha=0.9)
                # ax.legend(bbox_to_anchor=(1, -0.2), loc='upper right')
            else:
                ax.text(0.5, 0.5, f'No data for {jn}', ha='center', va='center', transform=ax.transAxes)
                ax.set_title(f'{jn}', fontsize=12, fontweight='bold', loc='left')
                ax.grid(True)
                
        # 最下部のみX軸ラベルを表示
        axs[-1].set_xlabel('Time [s]', fontsize=11, fontweight='bold')

        # 最下部のみ凡例を表示
        axs[-1].legend(bbox_to_anchor=(1, -0.2), loc='upper right')
        
        plt.suptitle('Joint Angle Report (Target vs Actual)', fontsize=16, fontweight='bold', y=0.98)
        fig.tight_layout(rect=[0, 0, 1, 0.96])
        
        # 画像として保存
        output_path = os.path.join(output_dir, "joint_angles_plot.png")
        fig.savefig(output_path, dpi=150, bbox_inches='tight')
        plt.close(fig)
        
        self.get_logger().info(f"✓ Saved joint tracking plot: {output_path}")

def main(args=None):
    rclpy.init(args=args)
    node = AnalysisPlotterNode()
    # 処理実行後に速やかにノードを破棄して終了する
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == "__main__":
    main()
