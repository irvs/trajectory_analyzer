#!/usr/bin/env python3
"""
Link padding解析専用モジュール
Plan軌道とFeedback軌道を比較してlink_padding推奨値を計算
"""
import os
import csv
import math
from typing import List, Tuple, Optional, Dict
import numpy as np
import matplotlib.pyplot as plt
import yaml

# FK計算用（PyKDL）
try:
    import kdl_parser.urdf as kdl_urdf
    import PyKDL
    HAS_KDL = True
except ImportError:
    HAS_KDL = False
    print("Warning: PyKDL or kdl_parser not available. FK-based analysis disabled.")


# ===== 解析対象リンク（リアルタイム処理と後処理で共通） =====
TARGET_LINKS = ['body_link', 'boom_link', 'arm_link', 'bucket_link', 'bucket_end_link']


# ===== FK Solver クラス =====

class FKSolver:
    """Forward Kinematics ソルバー（URDF から動的に構築）"""
    
    def __init__(self, urdf_path: str, base_link: str = "base_link", tip_link: str = "bucket_end_link"):
        self.ready = False
        self.urdf_path = urdf_path
        self.base_link = base_link
        self.tip_link = tip_link
        self._fk_solver = None
        self._chain = None
        self._chain_joint_names = []
        self._tree = None  # ★追加：ツリー全体を保持
        
        if not HAS_KDL:
            print("FKSolver: PyKDL not available")
            return
        
        if not urdf_path or not os.path.exists(urdf_path):
            print(f"FKSolver: URDF not found: {urdf_path}")
            return
        
        try:
            ok, tree = kdl_urdf.treeFromFile(urdf_path)
            if not ok:
                print("FKSolver: Failed to parse URDF")
                return
            
            self._tree = tree  # ★保存
            
            chain = tree.getChain(base_link, tip_link)
            if chain.getNrOfSegments() == 0:
                print(f"FKSolver: Empty chain from {base_link} to {tip_link}")
                return
            
            self._chain = chain
            self._fk_solver = PyKDL.ChainFkSolverPos_recursive(chain)
            
            # チェーンの関節名を抽出
            joint_names = []
            for i in range(chain.getNrOfSegments()):
                seg = chain.getSegment(i)
                jnt = seg.getJoint()
                name = jnt.getName()
                if name and name != "base_joint":
                    joint_names.append(name)
            self._chain_joint_names = joint_names
            
            self.ready = True
            print(f"FKSolver: Ready ({len(self._chain_joint_names)} joints: {', '.join(self._chain_joint_names)})")
            
        except Exception as e:
            print(f"FKSolver: Initialization failed: {e}")
            self.ready = False
    
    def compute(self, joint_positions: Dict[str, float]) -> Optional[Tuple[float, float, float]]:
        if not self.ready:
            return None
        
        try:
            nj = len(self._chain_joint_names)
            q = PyKDL.JntArray(nj)
            
            for k, jname in enumerate(self._chain_joint_names):
                if jname in joint_positions:
                    q[k] = joint_positions[jname]
                else:
                    q[k] = 0.0  # 見つからない場合は0
            
            frame = PyKDL.Frame()
            ret = self._fk_solver.JntToCart(q, frame)
            
            if ret >= 0:
                return (frame.p[0], frame.p[1], frame.p[2])
            else:
                return None
        except Exception:
            return None
    
    def compute_all_links(self, joint_positions: Dict[str, float]) -> Dict[str, Tuple[float, float, float]]:
        if not self.ready or not self._tree:
            return {}
        
        result = {}
        
        # ★共通定数を使用
        try:
            for link_name in TARGET_LINKS:
                # base_link から link_name までのチェーンを取得
                chain = self._tree.getChain(self.base_link, link_name)
                if chain.getNrOfSegments() == 0:
                    continue
                
                # このチェーンの関節名を取得
                chain_joint_names = []
                for i in range(chain.getNrOfSegments()):
                    seg = chain.getSegment(i)
                    jnt = seg.getJoint()
                    name = jnt.getName()
                    if name and name != "base_joint":
                        chain_joint_names.append(name)
                
                # FK計算
                nj = len(chain_joint_names)
                q = PyKDL.JntArray(nj)
                
                for k, jname in enumerate(chain_joint_names):
                    if jname in joint_positions:
                        q[k] = joint_positions[jname]
                    else:
                        q[k] = 0.0
                
                fk_solver = PyKDL.ChainFkSolverPos_recursive(chain)
                frame = PyKDL.Frame()
                ret = fk_solver.JntToCart(q, frame)
                
                if ret >= 0:
                    result[link_name] = (frame.p[0], frame.p[1], frame.p[2])
                    
        except Exception as e:
            print(f"Warning: compute_all_links failed: {e}")
        
        return result
    
    def get_joint_names(self) -> List[str]:
        return self._chain_joint_names.copy()


class TrajectoryAnalyzer:
    """軌道解析クラス（link_padding解析専用）"""
    
    def __init__(self):
        """シンプルなアナライザー"""
        pass


# ===== CSV保存関数（リアルタイム処理と後処理で共通） =====

def save_data_csv(data: Dict, csv_path: str, joint_names: List[str]):
    """
    データをdata.csvに保存（関節 + EE + ee_dist_err）
    
    Args:
        data: load_data_from_csv()と同じ形式の辞書
        csv_path: 保存先パス
        joint_names: 保存する関節名のリスト
    """
    # EEが保存可能か（load_data_from_csvが読む形式に合わせる）
    has_ee = (
        isinstance(data.get('ee'), dict) and
        isinstance(data['ee'].get('pos'), dict) and
        all(k in data['ee']['pos'] for k in ['ref', 'fb', 'err']) and
        len(data['ee']['pos']['ref']) == 3 and
        len(data['ee']['pos']['fb']) == 3 and
        len(data['ee']['pos']['err']) == 3 and
        len(data['ee']['pos']['ref'][0]) == len(data.get('t', []))
    )
    has_ee_dist_err = (isinstance(data.get('ee_dist_err'), list) and
                       len(data['ee_dist_err']) == len(data.get('t', [])))

    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)

        # ヘッダー作成
        header = ['t']
        for jn in joint_names:
            header += [f'{jn}_ref', f'{jn}_fb', f'{jn}_err', f'{jn}_vel']

        # ★EE列を追加（load_data_from_csv互換の名前）
        if has_ee:
            header += [
                'ee_ref_x', 'ee_ref_y', 'ee_ref_z',
                'ee_fb_x',  'ee_fb_y',  'ee_fb_z',
                'ee_err_x', 'ee_err_y', 'ee_err_z',
            ]

        # ★距離誤差列
        if has_ee_dist_err:
            header += ['ee_dist_err']

        writer.writerow(header)

        # データ行
        for i, t in enumerate(data['t']):
            row = [f'{t:.9f}']

            for jn in joint_names:
                if jn in data['joints']:
                    joint_data = data['joints'][jn]
                    row.append(joint_data['ref'][i] if i < len(joint_data['ref']) else 0.0)
                    row.append(joint_data['fb'][i] if i < len(joint_data['fb']) else 0.0)
                    row.append(joint_data['err'][i] if i < len(joint_data['err']) else 0.0)
                    row.append(joint_data['vel'][i] if i < len(joint_data['vel']) else 0.0)
                else:
                    row.extend([0.0, 0.0, 0.0, 0.0])

            # ★EE位置を追加
            if has_ee:
                # data['ee']['pos'][kind][axis][i]
                ref = data['ee']['pos']['ref']
                fb  = data['ee']['pos']['fb']
                err = data['ee']['pos']['err']
                row.extend([ref[0][i], ref[1][i], ref[2][i],
                            fb[0][i],  fb[1][i],  fb[2][i],
                            err[0][i], err[1][i], err[2][i]])

            # ★距離誤差を追加
            if has_ee_dist_err:
                row.append(data['ee_dist_err'][i])

            writer.writerow(row)


def save_plan_csv(plan_data: Dict, csv_path: str, joint_names: List[str], urdf_path: str = None):
    """
    Plan軌道をplan.csvに保存（EE位置も含む）
    
    Args:
        plan_data: Planデータ辞書
        csv_path: 保存先パス
        joint_names: 保存する関節名のリスト
        urdf_path: URDF パス（EE位置計算用、Noneの場合は計算しない）
    """
    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        
        # EE位置があるか確認
        has_ee = (plan_data.get('ee') and 
                 len(plan_data['ee']) == 3 and 
                 len(plan_data['ee'][0]) == len(plan_data['t']))
        
        # ヘッダー作成
        header = ['t'] + joint_names
        if has_ee:
            header += ['ee_x', 'ee_y', 'ee_z']
        
        writer.writerow(header)
        
        # データ行
        for i, t in enumerate(plan_data['t']):
            row = [f'{t:.9f}']
            
            for jn in joint_names:
                if jn in plan_data['joints'] and i < len(plan_data['joints'][jn]):
                    row.append(plan_data['joints'][jn][i])
                else:
                    row.append(0.0)
            
            # EE位置を追加
            if has_ee and i < len(plan_data['ee'][0]):
                row.extend([
                    plan_data['ee'][0][i],
                    plan_data['ee'][1][i],
                    plan_data['ee'][2][i]
                ])
            
            writer.writerow(row)


def compute_plan_ee_positions(plan_data: Dict, urdf_path: str, base_link: str = "base_link", 
                              tip_link: str = "bucket_end_link") -> List[Tuple[float, float, float]]:
    """
    Plan軌道の各点でFKを計算してEE位置を取得
    
    Args:
        plan_data: Planデータ辞書
        urdf_path: URDFファイルパス
        base_link: ベースリンク名
        tip_link: 先端リンク名
    
    Returns:
        EE位置のリスト [(x, y, z), ...]
    """
    fk_solver = FKSolver(urdf_path, base_link, tip_link)
    ee_positions = []
    
    if not fk_solver.ready:
        print("Warning: FK solver not ready, EE positions will not be computed")
        return ee_positions
    
    chain_joint_names = fk_solver.get_joint_names()
    
    for i in range(len(plan_data['t'])):
        # 関節角度を辞書形式で準備
        joint_positions = {}
        for chain_jn in chain_joint_names:
            if chain_jn in plan_data['joints'] and i < len(plan_data['joints'][chain_jn]):
                joint_positions[chain_jn] = plan_data['joints'][chain_jn][i]
            else:
                joint_positions[chain_jn] = 0.0
        
        # FK計算
        ee_pos = fk_solver.compute(joint_positions)
        ee_positions.append(ee_pos if ee_pos else (float('nan'), float('nan'), float('nan')))
    
    return ee_positions


def load_data_from_csv(csv_path: str) -> Dict:
    """CSVファイルからデータを読み込む（bucket_end_jointは除外）"""
    data = {
        't': [],
        'joints': {},
        'ee': {},
        'ee_dist_err': []
    }
    
    # 除外する関節名（FKで使わず、遅れ推定で異常値を出すため）
    EXCLUDED_JOINTS = ['bucket_end_joint']
    
    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        headers = reader.fieldnames
        
        # 関節名を抽出（bucket_end_jointは除外）
        joint_names = []
        for h in headers:
            if h.endswith('_ref') and not h.startswith('ee_'):
                joint_name = h[:-4]  # _refを除去
                if joint_name not in EXCLUDED_JOINTS:
                    joint_names.append(joint_name)
        
        for joint_name in joint_names:
            data['joints'][joint_name] = {
                'ref': [], 'fb': [], 'err': [], 'vel': []
            }
        
        # EE位置データ
        if 'ee_ref_x' in headers:
            data['ee']['pos'] = {
                'ref': [[], [], []],  # x, y, z
                'fb': [[], [], []],
                'err': [[], [], []]
            }
        
        # データ読み込み
        for row in reader:
            data['t'].append(float(row['t']))
            
            for joint_name in joint_names:
                data['joints'][joint_name]['ref'].append(float(row[f'{joint_name}_ref']))
                data['joints'][joint_name]['fb'].append(float(row[f'{joint_name}_fb']))
                data['joints'][joint_name]['err'].append(float(row[f'{joint_name}_err']))
                if f'{joint_name}_vel' in row:
                    data['joints'][joint_name]['vel'].append(float(row[f'{joint_name}_vel']))
            
            # EE位置
            if 'ee_ref_x' in headers:
                for i, axis in enumerate(['x', 'y', 'z']):
                    data['ee']['pos']['ref'][i].append(float(row[f'ee_ref_{axis}']))
                    data['ee']['pos']['fb'][i].append(float(row[f'ee_fb_{axis}']))
                    data['ee']['pos']['err'][i].append(float(row[f'ee_err_{axis}']))
            
            if 'ee_dist_err' in row:
                data['ee_dist_err'].append(float(row['ee_dist_err']))
    
    return data


def load_plan_from_csv(csv_path: str) -> Dict:
    """plan.csvからPlanデータを読み込む"""
    plan_data = {
        't': [],
        'joints': {},
        'ee': [[], [], []]  # x, y, z
    }
    
    if not os.path.exists(csv_path):
        return plan_data
    
    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        headers = reader.fieldnames
        
        # 関節名を抽出
        joint_names = [h for h in headers if h not in ['t', 'ee_x', 'ee_y', 'ee_z']]
        
        for joint_name in joint_names:
            plan_data['joints'][joint_name] = []
        
        for row in reader:
            plan_data['t'].append(float(row['t']))
            
            for joint_name in joint_names:
                if joint_name in row:
                    plan_data['joints'][joint_name].append(float(row[joint_name]))
            
            if 'ee_x' in row and 'ee_y' in row and 'ee_z' in row:
                try:
                    x = float(row['ee_x'])
                    y = float(row['ee_y'])
                    z = float(row['ee_z'])
                    if not (math.isnan(x) or math.isnan(y) or math.isnan(z)):
                        plan_data['ee'][0].append(x)
                        plan_data['ee'][1].append(y)
                        plan_data['ee'][2].append(z)
                except ValueError:
                    pass
    
    return plan_data


def _create_link_padding_summary_plot(stats: Dict, 
                                      link_correspondences: Dict,
                                      target_links: List[str],
                                      output_path: str):
    """Link padding解析結果を1枚のPNGにまとめて可視化"""
    fig = plt.figure(figsize=(20, 10))
    gs = fig.add_gridspec(2, 4, height_ratios=[1, 1.2], hspace=0.3, wspace=0.3)
    
    # ===== 上段：統計棒グラフ =====
    ax_stats = fig.add_subplot(gs[0, :])
    
    links_with_data = [link for link in target_links if link in stats]
    
    if links_with_data:
        x_pos = np.arange(len(links_with_data))
        max_errs = [stats[link]['max_error'] * 1000 for link in links_with_data]  # mm
        mean_errs = [stats[link]['mean_error'] * 1000 for link in links_with_data]
        p99_errs = [stats[link]['99percentile'] * 1000 for link in links_with_data]
        recommended = [stats[link]['recommended_padding'] * 1000 for link in links_with_data]
        
        width = 0.2
        ax_stats.bar(x_pos - 1.5*width, max_errs, width, label='Max error', color='red', alpha=0.7)
        ax_stats.bar(x_pos - 0.5*width, p99_errs, width, label='99%ile error', color='orange', alpha=0.7)
        ax_stats.bar(x_pos + 0.5*width, mean_errs, width, label='Mean error', color='blue', alpha=0.7)
        ax_stats.bar(x_pos + 1.5*width, recommended, width, label='Recommended padding', color='green', alpha=0.7)
        
        ax_stats.set_xlabel('Link', fontsize=12, fontweight='bold')
        ax_stats.set_ylabel('Deviation [mm]', fontsize=12, fontweight='bold')
        ax_stats.set_title('Link Padding Analysis - Statistical Summary', fontsize=14, fontweight='bold')
        ax_stats.set_xticks(x_pos)
        ax_stats.set_xticklabels(links_with_data, rotation=15, ha='right')
        ax_stats.legend(loc='upper left', fontsize=10)
        ax_stats.grid(True, alpha=0.3, axis='y')
        
        # 値をバーの上に表示
        for i, link in enumerate(links_with_data):
            ax_stats.text(i, recommended[i] + 2, f'{recommended[i]:.1f}', 
                         ha='center', va='bottom', fontsize=8, fontweight='bold')
    
    # ===== 下段左2つ：ロボット骨格（最大誤差時の姿勢） =====
    # 各リンクの最大誤差点を取得
    max_error_poses = {}
    for link_name in target_links:
        correspondences = link_correspondences.get(link_name, [])
        if correspondences:
            distances = [corr[2] for corr in correspondences]
            max_idx = distances.index(max(distances))
            max_error_poses[link_name] = {
                'plan': correspondences[max_idx][0],
                'fb': correspondences[max_idx][1],
                'distance': correspondences[max_idx][2]
            }
    
    # XY平面図（ロボット骨格）
    ax_xy_skel = fig.add_subplot(gs[1, 0])
    _draw_robot_skeleton(ax_xy_skel, max_error_poses, target_links, 0, 1, 'X [m]', 'Y [m]', 'XY view')
    
    # XZ平面図（ロボット骨格）
    ax_xz_skel = fig.add_subplot(gs[1, 1])
    _draw_robot_skeleton(ax_xz_skel, max_error_poses, target_links, 0, 2, 'X [m]', 'Z [m]', 'XZ view')
    
    # ===== 下段右2つ：bucket_end_linkの軌跡比較 =====
    detail_link = 'bucket_end_link' if 'bucket_end_link' in link_correspondences else target_links[-1]
    correspondences = link_correspondences.get(detail_link, [])
    
    if correspondences:
        plan_points = np.array([corr[0] for corr in correspondences])
        fb_points = np.array([corr[1] for corr in correspondences])
        distances = [corr[2] for corr in correspondences]
        
        views = [
            ('XY', 0, 1, 'X [m]', 'Y [m]', gs[1, 2]),
            ('XZ', 0, 2, 'X [m]', 'Z [m]', gs[1, 3])
        ]
        
        for view_name, idx1, idx2, xlabel, ylabel, grid_spec in views:
            ax = fig.add_subplot(grid_spec)
            
            # Plan軌跡（青）
            ax.plot(plan_points[:, idx1], plan_points[:, idx2], 
                   'o-', color='blue', markersize=3, linewidth=1.5, 
                   label='Plan trajectory', alpha=0.7)
            
            # Feedback軌跡（緑）
            ax.plot(fb_points[:, idx1], fb_points[:, idx2], 
                   'o-', color='green', markersize=3, linewidth=1.5, 
                   label='Feedback trajectory', alpha=0.7)
            
            # 対応線（灰色）- サンプリングして表示
            step = max(1, len(correspondences) // 30)
            for i in range(0, len(correspondences), step):
                plan_pos, fb_pos, dist = correspondences[i]
                ax.plot([plan_pos[idx1], fb_pos[idx1]], 
                       [plan_pos[idx2], fb_pos[idx2]], 
                       '-', color='gray', linewidth=0.5, alpha=0.3)
            
            # 最大誤差点を強調（赤）
            max_idx = distances.index(max(distances))
            max_plan = correspondences[max_idx][0]
            max_fb = correspondences[max_idx][1]
            max_dist = correspondences[max_idx][2]
            
            ax.plot([max_plan[idx1], max_fb[idx1]], 
                   [max_plan[idx2], max_fb[idx2]], 
                   '-', color='red', linewidth=2.5, alpha=0.9, 
                   label=f'Max: {max_dist*1000:.1f}mm')
            ax.plot(max_plan[idx1], max_plan[idx2], 'r*', markersize=15, markeredgecolor='darkred', markeredgewidth=1)
            ax.plot(max_fb[idx1], max_fb[idx2], 'r*', markersize=15, markeredgecolor='darkred', markeredgewidth=1)
            
            ax.set_xlabel(xlabel, fontsize=11)
            ax.set_ylabel(ylabel, fontsize=11)
            ax.set_title(f'{detail_link} - {view_name} view', fontsize=12, fontweight='bold')
            ax.grid(True, alpha=0.3)
            ax.legend(loc='best', fontsize=9)
            ax.axis('equal')
    
    plt.suptitle('Link Padding Analysis Report', fontsize=16, fontweight='bold', y=0.98)
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    
    print(f"\n✓ Saved link padding summary plot: {output_path}")


def _draw_robot_skeleton(ax, max_error_poses: Dict, target_links: List[str], 
                        idx1: int, idx2: int, xlabel: str, ylabel: str, title: str):
    """
    ロボット骨格を描画（最大誤差時の姿勢）
    
    Args:
        ax: matplotlib軸
        max_error_poses: 各リンクの最大誤差時の姿勢データ
        target_links: 描画するリンクのリスト
        idx1, idx2: 表示する軸のインデックス（0=X, 1=Y, 2=Z）
        xlabel, ylabel: 軸ラベル
        title: グラフタイトル
    """
    if not max_error_poses:
        ax.text(0.5, 0.5, 'No data', ha='center', va='center', transform=ax.transAxes)
        ax.set_title(title, fontsize=12, fontweight='bold')
        return
    
    # 原点を追加
    origin = (0.0, 0.0, 0.0)
    
    # Plan姿勢の座標を取得
    plan_coords = [origin]
    for link in target_links:
        if link in max_error_poses:
            plan_coords.append(max_error_poses[link]['plan'])
    
    # Feedback姿勢の座標を取得
    fb_coords = [origin]
    for link in target_links:
        if link in max_error_poses:
            fb_coords.append(max_error_poses[link]['fb'])
    
    # Plan骨格（青）
    if len(plan_coords) > 1:
        plan_x = [coord[idx1] for coord in plan_coords]
        plan_y = [coord[idx2] for coord in plan_coords]
        ax.plot(plan_x, plan_y, 'o-', color='blue', linewidth=3, 
               markersize=8, label='Plan pose', alpha=0.8, markeredgecolor='darkblue', markeredgewidth=1.5)
        
        # リンク名を表示
        for i, link in enumerate(['base'] + target_links[:len(plan_coords)-1]):
            if i < len(plan_coords):
                ax.text(plan_x[i], plan_y[i], f'  {link}', fontsize=8, color='blue', 
                       ha='left', va='bottom', fontweight='bold')
    
    # Feedback骨格（緑）
    if len(fb_coords) > 1:
        fb_x = [coord[idx1] for coord in fb_coords]
        fb_y = [coord[idx2] for coord in fb_coords]
        ax.plot(fb_x, fb_y, 's-', color='green', linewidth=3, 
               markersize=8, label='Feedback pose', alpha=0.8, markeredgecolor='darkgreen', markeredgewidth=1.5)
    
    # 誤差矢印（赤）
    for i, link in enumerate(target_links):
        if link in max_error_poses:
            plan_pos = max_error_poses[link]['plan']
            fb_pos = max_error_poses[link]['fb']
            distance = max_error_poses[link]['distance']
            
            # 矢印
            ax.annotate('', xy=(fb_pos[idx1], fb_pos[idx2]), 
                       xytext=(plan_pos[idx1], plan_pos[idx2]),
                       arrowprops=dict(arrowstyle='->', color='red', lw=2, alpha=0.7))
            
            # 誤差の数値表示（矢印の中点）
            mid_x = (plan_pos[idx1] + fb_pos[idx1]) / 2
            mid_y = (plan_pos[idx2] + fb_pos[idx2]) / 2
            ax.text(mid_x, mid_y, f'{distance*1000:.1f}mm', 
                   fontsize=9, color='red', fontweight='bold',
                   bbox=dict(boxstyle='round,pad=0.3', facecolor='white', edgecolor='red', alpha=0.8))
    
    ax.set_xlabel(xlabel, fontsize=11)
    ax.set_ylabel(ylabel, fontsize=11)
    ax.set_title(f'Robot Skeleton at Max Error - {title}', fontsize=12, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best', fontsize=10)
    ax.axis('equal')


def analyze_link_padding(data: Dict,
                         plan_data: Dict,
                         analyzer: TrajectoryAnalyzer,
                         urdf_path: str,
                         output_dir: str,
                         base_link: str = "base_link") -> Dict:
    """各リンクごとの追従誤差を解析してlink_padding推奨値を計算"""
    if not HAS_KDL:
        print("PyKDL not available, skipping link padding analysis")
        return {}
    
    if not plan_data['t'] or not plan_data['joints']:
        print("No plan data available, skipping link padding analysis")
        return {}
    
    fk_solver = FKSolver(urdf_path, base_link, "bucket_end_link")
    if not fk_solver.ready:
        print("FK solver not ready, skipping link padding analysis")
        return {}
    
    print("\n=== Link Padding Analysis (Nearest Neighbor Method) ===")
    
    t_data = data['t']
    t_plan = plan_data['t']
    
    # ★共通定数を使用（リアルタイム処理と後処理で同じ結果）
    target_links = TARGET_LINKS
    
    # ===== Step 1: Plan軌道の各リンク位置を計算 =====
    print("Computing Plan trajectory link positions...")
    plan_link_positions = {link: [] for link in target_links}
    
    for i in range(len(t_plan)):
        plan_joints = {}
        for joint_name in plan_data['joints'].keys():
            if i < len(plan_data['joints'][joint_name]):
                plan_joints[joint_name] = plan_data['joints'][joint_name][i]
            else:
                plan_joints[joint_name] = 0.0
        
        plan_link_poses = fk_solver.compute_all_links(plan_joints)
        
        for link_name in target_links:
            if link_name in plan_link_poses:
                plan_link_positions[link_name].append(plan_link_poses[link_name])
            else:
                plan_link_positions[link_name].append(None)
    
    print(f"  Computed {len(t_plan)} Plan trajectory points")
    
    # ===== Step 2: Feedback軌跡の各リンク位置を計算 =====
    print("Computing Feedback trajectory link positions...")
    fb_link_positions = {link: [] for link in target_links}
    
    for i in range(len(t_data)):
        fb_joints = {}
        for joint_name in data['joints'].keys():
            if i < len(data['joints'][joint_name]['fb']):
                fb_joints[joint_name] = data['joints'][joint_name]['fb'][i]
            else:
                fb_joints[joint_name] = 0.0
        
        fb_link_poses = fk_solver.compute_all_links(fb_joints)
        
        for link_name in target_links:
            if link_name in fb_link_poses:
                fb_link_positions[link_name].append(fb_link_poses[link_name])
            else:
                fb_link_positions[link_name].append(None)
    
    print(f"  Computed {len(t_data)} Feedback trajectory points")
    
    # ===== Step 3: 各Plan点に対して最近傍のFeedback点を探し、距離を計算 =====
    print("Computing nearest neighbor distances...")
    link_errors = {link: [] for link in target_links}
    link_correspondences = {link: [] for link in target_links}  # 対応点を保存（可視化用）
    
    for link_name in target_links:
        plan_poses = [p for p in plan_link_positions[link_name] if p is not None]
        fb_poses = [p for p in fb_link_positions[link_name] if p is not None]
        
        if not plan_poses or not fb_poses:
            print(f"  {link_name}: No valid data")
            continue
        
        # 各Plan点に対して最近傍のFB点を探す
        for plan_pos in plan_poses:
            # 全FB点との距離を計算
            distances = []
            for fb_pos in fb_poses:
                dist = math.sqrt(
                    (plan_pos[0] - fb_pos[0])**2 +
                    (plan_pos[1] - fb_pos[1])**2 +
                    (plan_pos[2] - fb_pos[2])**2
                )
                distances.append(dist)
            
            # 最小距離 = Plan軌道からのはみ出し量
            min_dist = min(distances)
            min_idx = distances.index(min_dist)
            nearest_fb_pos = fb_poses[min_idx]
            
            link_errors[link_name].append(min_dist)
            link_correspondences[link_name].append((plan_pos, nearest_fb_pos, min_dist))
        
        print(f"  {link_name}: Computed {len(link_errors[link_name])} nearest neighbor distances")
    
    # ===== Step 4: 統計計算 =====
    stats = {}
    print("\n=== Link Padding Recommendations ===")
    
    for link_name in target_links:
        errors = link_errors[link_name]
        
        if not errors:
            print(f"{link_name}: No valid data")
            continue
        
        max_err = np.max(errors)
        mean_err = np.mean(errors)
        std_err = np.std(errors)
        p95_err = np.percentile(errors, 95)
        p99_err = np.percentile(errors, 99)
        
        # 推奨padding = 99パーセンタイル + 安全率20%
        recommended_padding = p99_err * 1.2
        
        stats[link_name] = {
            'max_error': float(max_err),
            'mean_error': float(mean_err),
            'std_error': float(std_err),
            '95percentile': float(p95_err),
            '99percentile': float(p99_err),
            'recommended_padding': float(recommended_padding)
        }
        
        print(f"\n{link_name}:")
        print(f"  Max error (deviation):     {max_err:.4f} m")
        print(f"  Mean error:                {mean_err:.4f} m")
        print(f"  Std error:                 {std_err:.4f} m")
        print(f"  95%ile error:              {p95_err:.4f} m")
        print(f"  99%ile error:              {p99_err:.4f} m")
        print(f"  Recommended link_padding:  {recommended_padding:.4f} m")
    
    # ===== Step 5: 結果をファイルに保存 =====
    
    # 1. YAML統計ファイル
    yaml_path = os.path.join(output_dir, 'link_padding_summary.yaml')
    with open(yaml_path, 'w') as f:
        yaml.dump(stats, f, default_flow_style=False)
    
    print(f"\nSaved summary: {yaml_path}")
    
    # 2. CSV統計ファイル（★追加）
    csv_stats_path = os.path.join(output_dir, 'link_padding_summary.csv')
    with open(csv_stats_path, 'w', newline='') as f:
        fieldnames = ['link_name', 'max_error_m', 'mean_error_m', 'std_error_m', 
                     '95percentile_m', '99percentile_m', 'recommended_padding_m']
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        
        for link_name in target_links:
            if link_name in stats:
                s = stats[link_name]
                writer.writerow({
                    'link_name': link_name,
                    'max_error_m': s['max_error'],
                    'mean_error_m': s['mean_error'],
                    'std_error_m': s['std_error'],
                    '95percentile_m': s['95percentile'],
                    '99percentile_m': s['99percentile'],
                    'recommended_padding_m': s['recommended_padding']
                })
    
    print(f"Saved summary CSV: {csv_stats_path}")
    
    # 3. 対応点CSVファイル（★追加：video再生用）
    csv_correspondence_path = os.path.join(output_dir, 'link_correspondence_nearest.csv')
    with open(csv_correspondence_path, 'w', newline='') as f:
        fieldnames = ['link_name', 'plan_x', 'plan_y', 'plan_z', 
                     'fb_nearest_x', 'fb_nearest_y', 'fb_nearest_z', 'distance_m']
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        
        for link_name in target_links:
            correspondences = link_correspondences.get(link_name, [])
            for plan_pos, fb_pos, dist in correspondences:
                writer.writerow({
                    'link_name': link_name,
                    'plan_x': plan_pos[0],
                    'plan_y': plan_pos[1],
                    'plan_z': plan_pos[2],
                    'fb_nearest_x': fb_pos[0],
                    'fb_nearest_y': fb_pos[1],
                    'fb_nearest_z': fb_pos[2],
                    'distance_m': dist
                })
    
    print(f"Saved correspondence CSV: {csv_correspondence_path}")
    
    # 4. 簡易テキストレポート
    report_path = os.path.join(output_dir, 'link_padding_report.txt')
    with open(report_path, 'w') as f:
        f.write("=" * 70 + "\n")
        f.write("Link Padding Analysis Report (Nearest Neighbor Method)\n")
        f.write("=" * 70 + "\n\n")
        f.write("Methodology:\n")
        f.write("  For each point on the Plan trajectory, find the nearest point\n")
        f.write("  on the Feedback trajectory. The distance represents how much\n")
        f.write("  the robot deviates from the planned path.\n\n")
        f.write("Recommended link_padding values:\n")
        f.write("-" * 70 + "\n\n")
        
        for link_name in target_links:
            if link_name in stats:
                s = stats[link_name]
                f.write(f"{link_name}:\n")
                f.write(f"  Max deviation:        {s['max_error']:.4f} m\n")
                f.write(f"  Mean deviation:       {s['mean_error']:.4f} m\n")
                f.write(f"  99%ile deviation:     {s['99percentile']:.4f} m\n")
                f.write(f"  Recommended padding:  {s['recommended_padding']:.4f} m\n")
                f.write("\n")
        
        f.write("-" * 70 + "\n")
        f.write("Note: Recommended padding = 99th percentile × 1.2 (safety factor)\n")
    
    print(f"Saved report: {report_path}")
    
    # 5. 統合プロット（★追加）
    summary_plot_path = os.path.join(output_dir, 'link_padding_summary.png')
    _create_link_padding_summary_plot(
        stats=stats,
        link_correspondences=link_correspondences,
        target_links=target_links,
        output_path=summary_plot_path
    )
    
    return stats
