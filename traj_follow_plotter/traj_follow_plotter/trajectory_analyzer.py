#!/usr/bin/env python3
"""
軌道解析・プロット・遅れ補正の共通モジュール
リアルタイム処理と後処理の両方で使用可能
"""
import os
import csv
import math
from typing import List, Tuple, Optional, Dict
import numpy as np
from scipy.interpolate import interp1d
from scipy import signal as scipy_signal
from scipy.optimize import minimize_scalar
import matplotlib.pyplot as plt
import yaml


def max_abs(xs: List[float]) -> float:
    """リストの最大絶対値を返す"""
    m = 0.0
    for v in xs:
        if v is None:
            continue
        try:
            if isinstance(v, float) and math.isnan(v):
                continue
        except Exception:
            pass
        av = abs(v)
        if av > m:
            m = av
    return m


def quat_to_rpy(x: float, y: float, z: float, w: float) -> Tuple[float, float, float]:
    """
    Quaternion (x,y,z,w) から Roll-Pitch-Yaw (rad) へ変換
    PyKDLのGetRPYが環境によって異なる挙動をするため、自前実装
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class TrajectoryAnalyzer:
    """軌道解析・遅れ補正・プロット生成クラス"""
    
    def __init__(self, 
                 max_lag_s: float = 5.0,
                 lag_method: str = "frequency",
                 phase_use_velocity: bool = False):
        """
        Args:
            max_lag_s: 最大遅れ時間[秒]
            lag_method: 遅れ推定手法 ("correlation", "dtw", "gradient", "adaptive_kalman", "frequency", "polynomial")
            phase_use_velocity: 位相推定に速度を使うか
        """
        self.max_lag_s = max_lag_s
        self.lag_method = lag_method
        self.phase_use_velocity = phase_use_velocity
        
        # 適応カルマンフィルタ用の状態
        self.kalman_lag_estimate = 0.0
        self.kalman_lag_variance = 1.0
        self.kalman_process_noise = 0.01
        self.kalman_measurement_noise = 0.1
    
    def _finite_pair(self, a: np.ndarray, b: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """有限値のペアのみを返す"""
        ok = np.isfinite(a) & np.isfinite(b)
        return a[ok], b[ok]
    
    def estimate_lag_correlation(self, ref: List[float], fb: List[float], dt_est: float) -> float:
        """相関ベースの遅れ推定"""
        ref_arr = np.asarray(ref, dtype=float)
        fb_arr = np.asarray(fb, dtype=float)
        
        ref_arr, fb_arr = self._finite_pair(ref_arr, fb_arr)
        if len(ref_arr) < 20:
            return 0.0
        
        # 正規化
        ref_arr = (ref_arr - np.mean(ref_arr)) / (np.std(ref_arr) + 1e-9)
        fb_arr = (fb_arr - np.mean(fb_arr)) / (np.std(fb_arr) + 1e-9)
        
        max_lag_samples = int(self.max_lag_s / dt_est)
        corr = np.correlate(ref_arr, fb_arr, mode="full")
        lags = np.arange(-len(fb_arr) + 1, len(ref_arr))
        
        m = (lags >= -max_lag_samples) & (lags <= max_lag_samples)
        corr_filtered = corr[m]
        lags_filtered = lags[m]
        
        if len(corr_filtered) == 0:
            return 0.0
        
        best_lag = -int(lags_filtered[np.argmax(corr_filtered)])
        if best_lag < 0:
            best_lag = 0
        
        return best_lag * dt_est
    
    def estimate_lag_frequency(self, ref: List[float], fb: List[float], dt_est: float) -> float:
        """周波数領域での位相差検出"""
        ref_arr = np.asarray(ref, dtype=float)
        fb_arr = np.asarray(fb, dtype=float)
        
        ref_arr, fb_arr = self._finite_pair(ref_arr, fb_arr)
        if len(ref_arr) < 50:
            return 0.0
        
        # デトレンド
        ref_arr = scipy_signal.detrend(ref_arr)
        fb_arr = scipy_signal.detrend(fb_arr)
        
        n = len(ref_arr)
        n_fft = 2 ** int(np.ceil(np.log2(n * 2)))
        
        # FFT
        ref_fft = np.fft.rfft(ref_arr, n=n_fft)
        fb_fft = np.fft.rfft(fb_arr, n=n_fft)
        
        # クロススペクトル
        cross_spectrum = ref_fft * np.conj(fb_fft)
        phase_diff = np.angle(cross_spectrum)
        freqs = np.fft.rfftfreq(n_fft, d=dt_est)
        power = np.abs(cross_spectrum)
        
        valid_idx = (freqs > 0.01) & (freqs < 1.0 / (2 * dt_est) * 0.9) & (power > np.percentile(power, 50))
        
        if np.sum(valid_idx) < 5:
            return 0.0
        
        lags = []
        weights = []
        
        for i in np.where(valid_idx)[0]:
            if freqs[i] > 0:
                lag_at_freq = -phase_diff[i] / (2 * np.pi * freqs[i])
                
                candidates = []
                for k in range(-2, 3):
                    candidate = lag_at_freq + k / freqs[i]
                    if 0 <= candidate <= self.max_lag_s:
                        candidates.append(candidate)
                
                if candidates:
                    best_candidate = min(candidates, key=lambda x: abs(x - self.max_lag_s / 2))
                    lags.append(best_candidate)
                    weights.append(power[i])
        
        if not lags:
            return 0.0
        
        lags = np.array(lags)
        weights = np.array(weights)
        
        # 外れ値除去
        q1, q3 = np.percentile(lags, [25, 75])
        iqr = q3 - q1
        mask = (lags >= q1 - 1.5 * iqr) & (lags <= q3 + 1.5 * iqr)
        
        if np.sum(mask) > 0:
            lags = lags[mask]
            weights = weights[mask]
        
        lag_sec = np.average(lags, weights=weights)
        return max(0.0, min(lag_sec, self.max_lag_s))
    
    def compute_compensated_error(self, 
                                   t: List[float],
                                   ref: List[float], 
                                   fb: List[float], 
                                   dt_est: float) -> Tuple[List[float], float, List[float]]:
        """
        遅れ補正済み誤差を計算
        
        Returns:
            (compensated_error, lag_sec, fb_shifted)
        """
        if self.lag_method == "frequency":
            lag_sec = self.estimate_lag_frequency(ref, fb, dt_est)
        else:  # "correlation"
            lag_sec = self.estimate_lag_correlation(ref, fb, dt_est)
        
        n = len(t)
        if lag_sec <= 0 or len(ref) != n or len(fb) != n:
            return [ref[i] - fb[i] if i < len(ref) and i < len(fb) else math.nan for i in range(n)], lag_sec, list(fb)
        
        # 補間を使ってfbを時間シフト
        t_arr = np.array(t)
        fb_arr = np.array(fb)
        ref_arr = np.array(ref)
        
        valid = np.isfinite(fb_arr) & np.isfinite(t_arr)
        if np.sum(valid) < 2:
            return [math.nan] * n, lag_sec, [math.nan] * n
        
        try:
            interp_func = interp1d(t_arr[valid], fb_arr[valid], 
                                   kind='linear', bounds_error=False, fill_value=math.nan)
            fb_shifted = interp_func(t_arr + lag_sec)
            
            compensated_err = [ref_arr[i] - fb_shifted[i] if np.isfinite(fb_shifted[i]) else math.nan 
                              for i in range(n)]
            
            fb_shifted_list = list(fb_shifted)
        except Exception:
            compensated_err = [math.nan] * n
            fb_shifted_list = [math.nan] * n
        
        return compensated_err, lag_sec, fb_shifted_list


def load_data_from_csv(csv_path: str) -> Dict:
    """CSVファイルからデータを読み込む"""
    data = {
        't': [],
        'joints': {},
        'ee': {},
        'ee_dist_err': []
    }
    
    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        headers = reader.fieldnames
        
        # 関節名を抽出
        joint_names = []
        for h in headers:
            if h.endswith('_ref') and not h.startswith('ee_'):
                joint_name = h[:-4]  # _refを除去
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


def create_plot(data: Dict, 
                plan_data: Dict,
                analyzer: TrajectoryAnalyzer,
                output_path: str,
                topic: str = "",
                field_label: str = ""):
    """プロットを作成して保存"""
    
    joint_names = list(data['joints'].keys())
    n_joint_rows = len(joint_names)
    
    add_ee_pos = 'pos' in data['ee'] and len(data['ee']['pos']['ref'][0]) > 0
    add_ee_dist = len(data['ee_dist_err']) > 0
    
    rows = n_joint_rows + (3 if add_ee_pos else 0) + (1 if add_ee_dist else 0)
    fig, axs = plt.subplots(rows, 2, sharex=True, squeeze=False, figsize=(11, 2.2 * rows))
    
    t = data['t']
    dt_est = np.mean(np.diff(t)) if len(t) > 1 else 0.02
    
    # --- 関節プロット ---
    for r, joint_name in enumerate(joint_names):
        axp = axs[r][0]
        axe = axs[r][1]
        
        joint_data = data['joints'][joint_name]
        ref = joint_data['ref']
        fb = joint_data['fb']
        err = joint_data['err']
        
        axp.plot(t, ref, color="blue", linestyle="-", label="reference", linewidth=1.5)
        axp.plot(t, fb, color="green", linestyle="--", label="feedback", linewidth=1.5, alpha=0.7)
        
        # Planプロット
        if joint_name in plan_data['joints'] and len(plan_data['joints'][joint_name]) > 0:
            axp.plot(plan_data['t'], plan_data['joints'][joint_name], 'o', 
                     color="orange", markersize=3, label="plan", alpha=0.7)
        
        # 遅れ補正
        comp_err, lag_sec, fb_shifted = analyzer.compute_compensated_error(t, ref, fb, dt_est)
        axp.plot(t, fb_shifted, color="cyan", linestyle=":", 
                 label=f"fb_shifted (+{lag_sec:.3f}s)", linewidth=2, alpha=0.8)
        
        axe.plot(t, err, color="red", linestyle="-", label="error", linewidth=1.5)
        axe.plot(t, comp_err, color="purple", linestyle="--",
                 label=f"compensated ({analyzer.lag_method})", linewidth=1.5)
        
        # 最大誤差
        max_err = max_abs(err)
        max_comp_err = max_abs(comp_err)
        
        axp.set_ylabel(f"{joint_name} pos")
        axe.set_ylabel(f"{joint_name} err")
        axp.grid(True)
        axe.grid(True)
        
        axe.text(0.02, 0.98, 
                 f"Max err: {max_err:.6f}\nMax comp-err: {max_comp_err:.6f}\nLag: {lag_sec:.3f}s", 
                 transform=axe.transAxes, verticalalignment='top',
                 bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5),
                 fontsize=8)
        
        if r == 0:
            axp.legend(loc="upper right", fontsize=8)
            axe.legend(loc="upper right", fontsize=8)
    
    r0 = n_joint_rows
    
    # --- EE位置プロット ---
    if add_ee_pos:
        labels = ["ee_x (m)", "ee_y (m)", "ee_z (m)"]
        add_ee_plan = len(plan_data['ee'][0]) > 0
        
        for i in range(3):
            r = r0 + i
            axp = axs[r][0]
            axe = axs[r][1]
            
            ee_ref = data['ee']['pos']['ref'][i]
            ee_fb = data['ee']['pos']['fb'][i]
            ee_err = data['ee']['pos']['err'][i]
            
            axp.plot(t, ee_ref, color="blue", linestyle="-", label="ee_reference", linewidth=1.5)
            axp.plot(t, ee_fb, color="green", linestyle="--", label="ee_feedback", linewidth=1.5, alpha=0.7)
            
            # Plan EE
            if add_ee_plan:
                axp.plot(plan_data['t'][:len(plan_data['ee'][i])], plan_data['ee'][i], 'o', 
                         color="orange", markersize=3, label="ee_plan", alpha=0.7)
            
            # 遅れ補正
            comp_err, lag_sec, ee_fb_shifted = analyzer.compute_compensated_error(t, ee_ref, ee_fb, dt_est)
            axp.plot(t, ee_fb_shifted, color="cyan", linestyle=":", 
                     label=f"ee_fb_shifted (+{lag_sec:.3f}s)", linewidth=2, alpha=0.8)
            
            axe.plot(t, ee_err, color="red", linestyle="-", label="ee_error", linewidth=1.5)
            axe.plot(t, comp_err, color="purple", linestyle="--", label=f"ee_compensated", linewidth=1.5)
            
            max_ee_err = max_abs(ee_err)
            max_ee_comp_err = max_abs(comp_err)
            
            axp.set_ylabel(labels[i])
            axe.set_ylabel(labels[i].replace("(m)", "err (m)"))
            axp.grid(True)
            axe.grid(True)
            
            axe.text(0.02, 0.98, 
                     f"Max err: {max_ee_err:.6f} m\nMax comp-err: {max_ee_comp_err:.6f} m\nLag: {lag_sec:.3f}s", 
                     transform=axe.transAxes, verticalalignment='top',
                     bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.5),
                     fontsize=8)
            
            if r == r0:
                axp.legend(loc="upper right", fontsize=8)
                axe.legend(loc="upper right", fontsize=8)
        
        r0 += 3
    
    # --- EE 3D距離誤差 ---
    if add_ee_dist:
        r = r0
        axp = axs[r][0]
        axe = axs[r][1]
        
        # 元の3D距離誤差
        axe.plot(t, data['ee_dist_err'], color="red", linestyle="-", label="3D distance error", linewidth=1.5)
        
        max_dist_err = max_abs(data['ee_dist_err'])
        
        # 遅れ補正後の3D距離誤差を計算
        compensated_3d_err = []
        if add_ee_pos:
            # 各軸の補正後のfeedbackから3D距離誤差を計算
            ee_fb_shifted = [[], [], []]  # x, y, z
            lag_sec = 0.0
            
            for i in range(3):
                ee_ref = data['ee']['pos']['ref'][i]
                ee_fb = data['ee']['pos']['fb'][i]
                _, lag_sec, fb_shifted = analyzer.compute_compensated_error(t, ee_ref, ee_fb, dt_est)
                ee_fb_shifted[i] = fb_shifted
            
            # 補正後の3D距離誤差を計算
            for idx in range(len(t)):
                try:
                    ex = data['ee']['pos']['ref'][0][idx] - ee_fb_shifted[0][idx]
                    ey = data['ee']['pos']['ref'][1][idx] - ee_fb_shifted[1][idx]
                    ez = data['ee']['pos']['ref'][2][idx] - ee_fb_shifted[2][idx]
                    
                    if math.isnan(ex) or math.isnan(ey) or math.isnan(ez):
                        compensated_3d_err.append(math.nan)
                    else:
                        dist = math.sqrt(ex**2 + ey**2 + ez**2)
                        compensated_3d_err.append(dist)
                except (IndexError, TypeError):
                    compensated_3d_err.append(math.nan)
            
            # 補正後の3D距離誤差をプロット
            axe.plot(t, compensated_3d_err, color="purple", linestyle="--", 
                     label=f"compensated 3D error", linewidth=1.5)
            
            max_comp_3d_err = max_abs(compensated_3d_err)
            
            axe.text(0.02, 0.98, 
                     f"Max 3D err: {max_dist_err:.6f} m\nMax comp 3D err: {max_comp_3d_err:.6f} m\nLag: {lag_sec:.3f}s", 
                     transform=axe.transAxes, verticalalignment='top',
                     bbox=dict(boxstyle='round', facecolor='lightcoral', alpha=0.5),
                     fontsize=8)
        else:
            axe.text(0.02, 0.98, f"Max 3D err: {max_dist_err:.6f} m", 
                     transform=axe.transAxes, verticalalignment='top',
                     bbox=dict(boxstyle='round', facecolor='lightcoral', alpha=0.5),
                     fontsize=8)
        
        axe.set_ylabel("ee 3D dist err (m)")
        axe.grid(True)
        axe.legend(loc="upper right", fontsize=8)
        
        axp.axis('off')
    
    axs[-1][0].set_xlabel("time (s)")
    axs[-1][1].set_xlabel("time (s)")
    
    title = f"{topic} ({field_label}) samples={len(t)} max_lag_s={analyzer.max_lag_s} method={analyzer.lag_method}"
    fig.suptitle(title)
    fig.tight_layout()
    
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
    
    return output_path
