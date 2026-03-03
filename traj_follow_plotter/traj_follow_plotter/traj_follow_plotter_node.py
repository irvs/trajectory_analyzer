#!/usr/bin/env python3
import os
import math
import signal
import subprocess
import datetime
import time
import tempfile
from typing import Optional, List, Tuple
from collections import OrderedDict

import numpy as np
from scipy.interpolate import interp1d
from scipy import signal as scipy_signal
from scipy.optimize import minimize_scalar

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse

from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState

# planの保存用（YAML化）
from rosidl_runtime_py.convert import message_to_ordereddict
import yaml

import matplotlib.pyplot as plt

# ★あなたのAction定義に合わせて import してください
# 例: package名が traj_recorder_msgs の場合
from traj_recorder_msgs.action import TrajFollow

# ===== FK/URDF 追加 (Humble向け: kdl_parser_py を使わず jvytee/kdl_parser を使用) =====
import kdl_parser.urdf as kdl_urdf
import PyKDL

import traceback


def pick_fields(msg: JointTrajectoryControllerState):
    """
    環境差対応:
      - reference / feedback / error
      - desired / actual / error
    """
    if hasattr(msg, "reference") and hasattr(msg, "feedback"):
        return msg.reference, msg.feedback, msg.error, "reference/feedback"
    if hasattr(msg, "desired") and hasattr(msg, "actual"):
        return msg.desired, msg.actual, msg.error, "desired/actual"
    raise RuntimeError(
        "Expected reference/feedback or desired/actual fields in JointTrajectoryControllerState."
    )


def max_abs(xs: List[float]) -> float:
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
    # ZYX (roll-pitch-yaw) 変換
    # roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class TrajFollowRecordActionServer(Node):
    """
    Action名: traj_follow_record

    - Goal受信で計測開始
      - /<controller>/state 購読開始（パラメータで指定）
      - plan.yaml 保存
      - ros2 bag record -a を同ディレクトリ配下に起動（オプションでON/OFF）

    - Cancel受信で計測終了
      - bag停止
      - plot.png / data.csv 保存
      - Result.ok を返す
    """

    def __init__(self):
        super().__init__("traj_follow_record_action_server")

        # URDFの実際の関節名を定義（zx200用）
        self.urdf_joint_names = [
            "swing_joint",
            "boom_joint",
            "arm_joint",
            "bucket_joint",
            "bucket_end_joint"
        ]

        self.t: List[float] = []
        self.ref: List[List[float]] = []
        self.fb:  List[List[float]] = []
        self.err: List[List[float]] = []
        self.vel: List[List[float]] = []

        from ament_index_python.packages import get_package_share_directory
        pkg_share = get_package_share_directory('traj_follow_plotter')
        default_output = os.path.join(pkg_share, "..", "..", "..", "data")
        default_urdf = os.path.join(pkg_share, "urdf", "zx200.urdf")

        self.declare_parameter("state_topic", "/zx200/upper_arm_controller/controller_state")
        self.declare_parameter("output_root", os.path.normpath(default_output))
        self.declare_parameter("record_bag_all", True)     # -a 相当をデフォルトで回すか

        self.declare_parameter("max_lag_s", 5.0)
        self.declare_parameter("phase_use_velocity", False)
        self.declare_parameter("lag_method", "frequency")  # "correlation", "dtw", "gradient", "adaptive_kalman", "frequency", "polynomial"

        # ===== FK/URDF 追加パラメータ =====
        self.declare_parameter("urdf_path", os.path.normpath(default_urdf))  # 相対パス: traj_follow_plotterから見て../urdf/
        self.declare_parameter("fk_base_link", "base_link")
        self.declare_parameter("fk_tip_link", "bucket_end_link")

        # ---- Action Server ----
        self._action_srv = ActionServer(
            self,
            TrajFollow,
            "traj_follow_record",
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
        )

        # ---- 実行中の状態 ----
        self._goal_handle = None
        self._sub = None

        self.started = False
        self.start_time: Optional[float] = None
        if len(self.t) > 1:
            self.dt_est = np.mean(np.diff(self.t))
        else:
            self.dt_est = 0.02  # fallback

        self.topic = str(self.get_parameter("state_topic").value)

        self.max_lag_s = float(self.get_parameter("max_lag_s").value)
        self.phase_use_velocity = bool(self.get_parameter("phase_use_velocity").value)

        # 出力ディレクトリ（Goalごとにサブディレクトリ）
        self.output_root = str(self.get_parameter("output_root").value)
        self.out_dir = ""
        self.out_png = ""
        self.out_csv = ""
        self.out_plan = ""
        self.out_plan_csv = ""
        self.bag_dir = ""

        # 記録データ
        self.field_label = None
        self.n_all = None
        self.plot_joints: List[int] = []  # 全関節を保存する（必要なら絞る）

        self.t: List[float] = []
        self.ref: List[List[float]] = []
        self.fb: List[List[float]] = []
        self.err: List[List[float]] = []
        self.vel: List[List[float]] = []

        # bag用
        self._bag_proc: Optional[subprocess.Popen] = None
        self._record_bag_all = bool(self.get_parameter("record_bag_all").value)

        # matplotlibは最後だけ描画
        plt.ioff()

        # ===== FK 状態追加 =====
        self._fk_ready = False
        self._fk_failed_reason = ""
        self._kdl_chain = None
        self._fk_solver = None
        self._chain_joint_names: List[str] = []
        self._joint_name_to_msg_index = {}

        # 刃先（EE）ログ: 位置(x,y,z)
        self.ee_ref = [[], [], []]
        self.ee_fb  = [[], [], []]
        self.ee_err = [[], [], []]

        # 刃先（EE）ログ: 姿勢（RPY: roll,pitch,yaw）
        self.ee_rpy_ref = [[], [], []]
        self.ee_rpy_fb  = [[], [], []]
        self.ee_rpy_err = [[], [], []]

        # 刃先（EE）ログ: 姿勢（Quaternion: x,y,z,w）※CSV用にも残す
        self.ee_quat_ref = [[], [], [], []]
        self.ee_quat_fb  = [[], [], [], []]

        # 刃先（EE）ログ: 3D距離誤差
        self.ee_dist_err = []  # sqrt(ex^2 + ey^2 + ez^2)

        # URDF 読み込み & FK チェーン準備（失敗しても計測自体は続行）
        self._init_fk_from_urdf()

        # ===== plan trajectory buffers =====
        self.plan_t = []
        self.plan_pos = []
        self.plan_joint_names = []  # Plan軌道の関節名を保存
        
        # ===== plan EE position buffers =====
        self.ee_plan = [[], [], []]  # x, y, z

        # ===== 適応カルマンフィルタ用の状態 =====
        self.kalman_lag_estimate = 0.0  # 現在の遅れ推定値
        self.kalman_lag_variance = 1.0  # 推定値の分散
        self.kalman_process_noise = 0.01  # プロセスノイズ（遅れの変動）
        self.kalman_measurement_noise = 0.1  # 観測ノイズ

    # ---------------------------
    # FK init
    # ---------------------------

    def _init_fk_from_urdf(self):
        urdf_path = str(self.get_parameter("urdf_path").value)
        base_link = str(self.get_parameter("fk_base_link").value)
        tip_link  = str(self.get_parameter("fk_tip_link").value)

        if not urdf_path:
            self._fk_failed_reason = "urdf_path is empty"
            self.get_logger().warn("FK disabled: urdf_path is empty")
            return

        if not os.path.exists(urdf_path):
            self._fk_failed_reason = f"urdf_path not found: {urdf_path}"
            self.get_logger().warn(f"FK disabled: URDF not found: {urdf_path}")
            return

        try:
            ok, tree = kdl_urdf.treeFromFile(urdf_path)
            if not ok:
                self._fk_failed_reason = "treeFromFile failed"
                self.get_logger().warn("FK disabled: treeFromFile failed")
                return

            chain = tree.getChain(base_link, tip_link)
            if chain.getNrOfSegments() == 0:
                self._fk_failed_reason = f"KDL chain empty: {base_link} -> {tip_link}"
                self.get_logger().warn(f"FK disabled: KDL chain empty: {base_link} -> {tip_link}")
                return

            self._kdl_chain = chain
            self._fk_solver = PyKDL.ChainFkSolverPos_recursive(chain)

            # チェーンに含まれる関節名（fixedは除外）
            joint_names = []
            for i in range(chain.getNrOfSegments()):
                seg = chain.getSegment(i)
                jnt = seg.getJoint()
                name = jnt.getName()
                # fixed joint は名前が空になることが多いので、それを除外
                if name and name != "base_joint":
                    joint_names.append(name)
            self._chain_joint_names = joint_names

            self._fk_ready = True
            self.get_logger().info(
                f"FK enabled: {base_link} -> {tip_link}, joints={len(self._chain_joint_names)}"
            )
        except Exception as e:
            self._fk_ready = False
            self._fk_failed_reason = str(e)
            self.get_logger().warn("FK disabled with exception:\n" + traceback.format_exc())

    # ---------------------------
    # Action callbacks
    # ---------------------------

    def goal_cb(self, goal_request: TrajFollow.Goal):
        # 同時実行は拒否
        if self._goal_handle is not None:
            self.get_logger().warn("Another goal is active; rejecting new goal.")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        return CancelResponse.ACCEPT

    async def execute_cb(self, goal_handle):
        self._goal_handle = goal_handle

        # 出力ディレクトリ作成（同一Goal内の成果物を全部ここへ）
        self._prepare_output_dir()

        # state購読開始前にバッファリセット
        self._reset_buffers()

        # plan（Goal内容）を即保存 ← _reset_buffers()の後に移動
        self._save_plan_yaml(goal_handle.request)

        self.topic = str(self.get_parameter("state_topic").value)
        self._sub = self.create_subscription(
            JointTrajectoryControllerState,
            self.topic,
            self.on_state,
            10,
        )

        # 計測開始
        self.started = True
        self.start_time = self.get_clock().now().nanoseconds * 1e-9

        # bag開始（-a相当）
        if self._record_bag_all:
            self._start_bag_record_all()

        fb = TrajFollow.Feedback()
        fb.status = f"recording: {self.topic}"
        goal_handle.publish_feedback(fb)

        # Cancelされるまで回す
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                fb.status = "cancel_requested: stopping"
                goal_handle.publish_feedback(fb)
                goal_handle.canceled()
                break

            # 少し回す（購読コールバックを回す）
            rclpy.spin_once(self, timeout_sec=0.1)

        # 停止処理
        self.started = False
        if self._sub is not None:
            self.destroy_subscription(self._sub)
            self._sub = None

        self._stop_bag()

        fb.status = "saving"
        goal_handle.publish_feedback(fb)

        ok, msg = self._finalize_and_save()

        result = TrajFollow.Result()
        result.ok = bool(ok)

        self.get_logger().info(msg)

        self._goal_handle = None
        return result

    # ---------------------------
    # Output directory & plan save
    # ---------------------------

    def _prepare_output_dir(self):
        os.makedirs(self.output_root, exist_ok=True)
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.out_dir = os.path.join(self.output_root, f"run_{stamp}")
        os.makedirs(self.out_dir, exist_ok=True)

        self.out_png = os.path.join(self.out_dir, "plot.png")
        self.out_csv = os.path.join(self.out_dir, "data.csv")
        self.out_plan = os.path.join(self.out_dir, "plan.yaml")
        self.out_plan_csv = os.path.join(self.out_dir, "plan.csv")
        self.bag_dir = os.path.join(self.out_dir, "bag")

        self.get_logger().info(f"Output dir: {self.out_dir}")


    def _to_plain(self, obj):
        """OrderedDictなどをYAMLで安全に書ける plain dict/list に変換"""
        if isinstance(obj, OrderedDict):
            return {k: self._to_plain(v) for k, v in obj.items()}
        if isinstance(obj, dict):
            return {k: self._to_plain(v) for k, v in obj.items()}
        if isinstance(obj, (list, tuple)):
            return [self._to_plain(v) for v in obj]
        return obj

    def _save_plan_yaml(self, goal_msg: TrajFollow.Goal):
        data = {
            "time_scaling": float(goal_msg.time_scaling),
            "velocity_scaling": float(goal_msg.velocity_scaling),
            "acceleration_scaling": float(goal_msg.acceleration_scaling),
            "plan": self._to_plain(message_to_ordereddict(goal_msg.plan)),
        }
        with open(self.out_plan, "w", encoding="utf-8") as f:
            yaml.safe_dump(data, f, sort_keys=False, allow_unicode=True)
        self.get_logger().info(f"Saved plan: {self.out_plan}")
        
        # ===== planから軌道データを抽出 =====
        self._extract_plan_trajectory(goal_msg.plan)

    def _extract_plan_trajectory(self, plan_trajectory):
        """planからtime_from_startと関節位置を抽出"""
        self.plan_t = []
        self.plan_pos = []
        self.plan_joint_names = []  # Plan軌道の関節名を保存
        
        try:
            # RobotTrajectory内のjoint_trajectoryを取得
            joint_traj = plan_trajectory.joint_trajectory
            
            # joint_namesを取得（関節の順序を把握）
            joint_names = list(joint_traj.joint_names)
            self.plan_joint_names = joint_names
            
            # 各pointから時刻と位置を抽出
            for point in joint_traj.points:
                # time_from_startをfloat秒に変換
                t_sec = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
                self.plan_t.append(t_sec)
                
                # 位置データを保存（全関節分）
                positions = list(point.positions)
                self.plan_pos.append(positions)
            
            if self.plan_t:
                self.get_logger().info(f"Extracted plan trajectory: {len(self.plan_t)} points")
                
                # ===== planからEE位置をFKで計算 =====
                self._compute_plan_ee_positions()
            else:
                self.get_logger().warn("No trajectory points found in plan")
                
        except Exception as e:
            self.get_logger().warn(f"Failed to extract plan trajectory: {e}\n{traceback.format_exc()}")
            self.plan_t = []
            self.plan_pos = []

    def _compute_plan_ee_positions(self):
        """Plan軌道の各点でFKを計算してEE位置を保存"""
        if not self._fk_ready or self._fk_solver is None:
            self.get_logger().warn("FK not ready; skipping plan EE position calculation")
            return
        
        if not self.plan_joint_names:
            self.get_logger().warn("Plan joint names not available; skipping plan EE position calculation")
            return
        
        self.ee_plan = [[], [], []]  # x, y, z
        
        try:
            nj = len(self._chain_joint_names)
            
            # Plan軌道のjoint_namesからchain_joint_namesへのマッピングを作成
            plan_to_chain_index = {}
            for k, chain_jn in enumerate(self._chain_joint_names):
                if chain_jn in self.plan_joint_names:
                    plan_to_chain_index[k] = self.plan_joint_names.index(chain_jn)
                else:
                    self.get_logger().warn(f"Chain joint '{chain_jn}' not found in plan joint names")
            
            for positions in self.plan_pos:
                q = PyKDL.JntArray(nj)
                
                # 関節角度をセット（plan_joint_namesの順序から変換）
                for k, chain_jn in enumerate(self._chain_joint_names):
                    if k in plan_to_chain_index:
                        plan_idx = plan_to_chain_index[k]
                        if plan_idx < len(positions):
                            q[k] = positions[plan_idx]
                        else:
                            q[k] = 0.0
                    else:
                        q[k] = 0.0
                
                # FK計算
                frame = PyKDL.Frame()
                ret = self._fk_solver.JntToCart(q, frame)
                
                if ret >= 0:
                    self.ee_plan[0].append(frame.p[0])  # x
                    self.ee_plan[1].append(frame.p[1])  # y
                    self.ee_plan[2].append(frame.p[2])  # z
                else:
                    self.ee_plan[0].append(math.nan)
                    self.ee_plan[1].append(math.nan)
                    self.ee_plan[2].append(math.nan)
            
            self.get_logger().info(f"Computed plan EE positions: {len(self.ee_plan[0])} points")
            
        except Exception as e:
            self.get_logger().warn(f"Failed to compute plan EE positions: {e}\n{traceback.format_exc()}")
            self.ee_plan = [[], [], []]

    # ---------------------------
    # Recording buffers
    # ---------------------------

    def _reset_buffers(self):
        self.field_label = None
        self.n_all = None
        self.plot_joints = []

        self.t = []
        self.ref = []
        self.fb = []
        self.err = []
        self.vel = []

        # ===== FK buffers reset =====
        self.ee_ref = [[], [], []]
        self.ee_fb  = [[], [], []]
        self.ee_err = [[], [], []]

        self.ee_rpy_ref = [[], [], []]
        self.ee_rpy_fb  = [[], [], []]
        self.ee_rpy_err = [[], [], []]

        self._joint_name_to_msg_index = {}
        
        # ===== plan trajectory buffers =====
        self.plan_t = []
        self.plan_pos = []
        self.plan_joint_names = []

    def _ensure_buffers(self, n_all: int):
        if self.n_all == n_all and self.ref:
            return
        self.n_all = n_all
        self.plot_joints = list(range(n_all))  # 全関節

        self.ref = [[] for _ in range(n_all)]
        self.fb  = [[] for _ in range(n_all)]
        self.err = [[] for _ in range(n_all)]
        self.vel = [[] for _ in range(n_all)]

    def on_state(self, msg: JointTrajectoryControllerState):
        if not self.started:
            return

        now = self.get_clock().now().nanoseconds * 1e-9

        try:
            ref_pt, fb_pt, err_pt, label = pick_fields(msg)
            self.field_label = label
        except Exception as e:
            self.get_logger().error(str(e))
            return

        n = len(getattr(ref_pt, "positions", []))
        if n == 0:
            return
        self._ensure_buffers(n)

        t_rel = now - (self.start_time or now)
        self.t.append(t_rel)

        # dt_est を更新（録画中に推定精度を上げる）
        if len(self.t) >= 2:
            dt = self.t[-1] - self.t[-2]
            if 1e-6 < dt < 1.0:
                self.dt_est = 0.98 * self.dt_est + 0.02 * dt if self.dt_est > 0 else dt

        fb_pos = list(getattr(fb_pt, "positions", []))
        fb_vel = list(getattr(fb_pt, "velocities", []))  # empty ok
        ref_pos = list(getattr(ref_pt, "positions", []))
        err_pos = list(getattr(err_pt, "positions", []))

        for j in range(n):
            self.ref[j].append(ref_pos[j] if j < len(ref_pos) else math.nan)
            self.fb[j].append(fb_pos[j] if j < len(fb_pos) else math.nan)
            self.err[j].append(err_pos[j] if j < len(err_pos) else math.nan)
            self.vel[j].append(fb_vel[j] if j < len(fb_vel) else math.nan)

        # ===== FK: joint_names マッピング（最初だけ）=====
        if self._fk_ready and not self._joint_name_to_msg_index:
            msg_joint_names = list(getattr(msg, "joint_names", []))
            if not msg_joint_names:
                self.get_logger().warn("FK disabled for this run: msg.joint_names is empty")
                self._fk_ready = False
            else:
                m = {}
                missing = []
                for jn in self._chain_joint_names:  # enumerate を削除
                    if jn in msg_joint_names:
                        m[jn] = msg_joint_names.index(jn)
                    else:
                        missing.append(jn)  # これで文字列になる
                if missing:
                    self.get_logger().warn(
                        "FK disabled for this run: chain joint(s) not in msg.joint_names: "
                        + ", ".join(missing)
                    )
                    self._fk_ready = False
                else:
                    self._joint_name_to_msg_index = m
                    self.get_logger().info("FK joint mapping is ready")

        # ===== FK: bucket_end_link の位置/姿勢を保存 =====
        if self._fk_ready and self._fk_solver is not None:
            try:
                nj = len(self._chain_joint_names)
                q_ref = PyKDL.JntArray(nj)
                q_fb  = PyKDL.JntArray(nj)

                for k, jn in enumerate(self._chain_joint_names):
                    idx = self._joint_name_to_msg_index[jn]
                    q_ref[k] = ref_pos[idx] if idx < len(ref_pos) else float("nan")
                    q_fb[k]  = fb_pos[idx] if idx < len(fb_pos) else float("nan")

                fr_ref = PyKDL.Frame()
                fr_fb  = PyKDL.Frame()

                ret1 = self._fk_solver.JntToCart(q_ref, fr_ref)
                ret2 = self._fk_solver.JntToCart(q_fb,  fr_fb)

                if ret1 >= 0 and ret2 >= 0:
                    # position
                    xr, yr, zr = fr_ref.p[0], fr_ref.p[1], fr_ref.p[2]
                    xf, yf, zf = fr_fb.p[0],  fr_fb.p[1],  fr_fb.p[2]

                    self.ee_ref[0].append(xr); self.ee_ref[1].append(yr); self.ee_ref[2].append(zr)
                    self.ee_fb[0].append(xf);  self.ee_fb[1].append(yf);  self.ee_fb[2].append(zf)
                    self.ee_err[0].append(xr - xf); self.ee_err[1].append(yr - yf); self.ee_err[2].append(zr - zf)

                    # 3D距離誤差
                    dist_err = math.sqrt((xr - xf)**2 + (yr - yf)**2 + (zr - zf)**2)
                    self.ee_dist_err.append(dist_err)

                    # Quaternion (x,y,z,w)
                    qrx, qry, qrz, qrw = fr_ref.M.GetQuaternion()
                    qfx, qfy, qfz, qfw = fr_fb.M.GetQuaternion()

                    # RPY (roll,pitch,yaw) を Quaternion から計算（PyKDLのGetRPYの環境差回避）
                    rr, pr, yr_ = quat_to_rpy(qrx, qry, qrz, qrw)
                    rf, pf, yf_ = quat_to_rpy(qfx, qfy, qfz, qfw)

                    self.ee_rpy_ref[0].append(rr); self.ee_rpy_ref[1].append(pr); self.ee_rpy_ref[2].append(yr_)
                    self.ee_rpy_fb[0].append(rf);  self.ee_rpy_fb[1].append(pf);  self.ee_rpy_fb[2].append(yf_)
                    self.ee_rpy_err[0].append(rr - rf); self.ee_rpy_err[1].append(pr - pf); self.ee_rpy_err[2].append(yr_ - yf_)

                    # Quaternion (x,y,z,w)
                    qrx, qry, qrz, qrw = fr_ref.M.GetQuaternion()
                    qfx, qfy, qfz, qfw = fr_fb.M.GetQuaternion()

                    self.ee_quat_ref[0].append(qrx); self.ee_quat_ref[1].append(qry); self.ee_quat_ref[2].append(qrz); self.ee_quat_ref[3].append(qrw)
                    self.ee_quat_fb[0].append(qfx);  self.ee_quat_fb[1].append(qfy);  self.ee_quat_fb[2].append(qfz);  self.ee_quat_fb[3].append(qfw)
                else:
                    for a in range(3):
                        self.ee_ref[a].append(math.nan)
                        self.ee_fb[a].append(math.nan)
                        self.ee_err[a].append(math.nan)

                        self.ee_rpy_ref[a].append(math.nan)
                        self.ee_rpy_fb[a].append(math.nan)
                        self.ee_rpy_err[a].append(math.nan)

                    for a in range(4):
                        self.ee_quat_ref[a].append(math.nan)
                        self.ee_quat_fb[a].append(math.nan)

                    self.ee_dist_err.append(math.nan)

            except Exception as e:
                self.get_logger().warn("FK failed during run; disabling FK.\n" + traceback.format_exc())
                self._fk_ready = False

    # ---------------------------
    # Phase lag estimation (improved)
    # ---------------------------

    def _finite_pair(self, a: np.ndarray, b: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        ok = np.isfinite(a) & np.isfinite(b)
        return a[ok], b[ok]

    def estimate_lag_dtw(self, ref: List[float], fb: List[float], max_lag_s: float) -> float:
        """DTWベースの遅れ推定（サンプル単位ではなく時間[秒]で返す）"""
        ref_arr = np.asarray(ref, dtype=float)
        fb_arr  = np.asarray(fb, dtype=float)

        ref_arr, fb_arr = self._finite_pair(ref_arr, fb_arr)
        if len(ref_arr) < 20:
            return 0.0

        # 正規化
        ref_arr = (ref_arr - np.mean(ref_arr)) / (np.std(ref_arr) + 1e-9)
        fb_arr  = (fb_arr - np.mean(fb_arr)) / (np.std(fb_arr) + 1e-9)

        n = len(ref_arr)
        max_lag = int(max(0.0, max_lag_s) / self.dt_est)
        
        # 簡易DTW: Sakoe-Chiba band制約付き
        window = min(max_lag, n // 2)
        dtw_matrix = np.full((n, n), np.inf)
        dtw_matrix[0, 0] = 0.0

        for i in range(1, n):
            for j in range(max(1, i - window), min(n, i + window + 1)):
                cost = (ref_arr[i] - fb_arr[j]) ** 2
                dtw_matrix[i, j] = cost + min(
                    dtw_matrix[i-1, j],      # insertion
                    dtw_matrix[i, j-1],      # deletion
                    dtw_matrix[i-1, j-1]     # match
                )

        # バックトレースしてマッチングを取得
        i, j = n - 1, n - 1
        path_i, path_j = [i], [j]
        
        while i > 0 and j > 0:
            candidates = [
                (i-1, j, dtw_matrix[i-1, j]),
                (i, j-1, dtw_matrix[i, j-1]),
                (i-1, j-1, dtw_matrix[i-1, j-1])
            ]
            candidates = [(ii, jj, val) for ii, jj, val in candidates if np.isfinite(val)]
            if not candidates:
                break
            
            i, j, _ = min(candidates, key=lambda x: x[2])
            path_i.append(i)
            path_j.append(j)

        # 平均的な遅れを計算
        lag_samples = np.mean([path_i[k] - path_j[k] for k in range(len(path_i))])
        lag_sec = lag_samples * self.dt_est
        
        self.get_logger().info(f"DTW lag estimation: {lag_sec:.3f}s ({lag_samples:.1f} samples)")
        return max(0.0, lag_sec)

    def estimate_lag_adaptive_kalman(self, ref: List[float], fb: List[float], max_lag_s: float) -> float:
        """
        適応カルマンフィルタによる遅れ推定
        遅れが時間変動する場合に有効
        """
        ref_arr = np.asarray(ref, dtype=float)
        fb_arr  = np.asarray(fb, dtype=float)

        ref_arr, fb_arr = self._finite_pair(ref_arr, fb_arr)
        if len(ref_arr) < 50:
            return 0.0

        # 正規化
        ref_arr = (ref_arr - np.mean(ref_arr)) / (np.std(ref_arr) + 1e-9)
        fb_arr  = (fb_arr - np.mean(fb_arr)) / (np.std(fb_arr) + 1e-9)

        # ウィンドウサイズ（局所的な遅れ推定）
        window_size = min(100, len(ref_arr) // 5)
        max_lag_samples = int(max_lag_s / self.dt_est)

        lag_estimates = []
        
        # スライディングウィンドウで遅れを推定
        for i in range(window_size, len(ref_arr), window_size // 2):
            ref_window = ref_arr[max(0, i - window_size):i]
            fb_window  = fb_arr[max(0, i - window_size):i]
            
            if len(ref_window) < 20:
                continue
            
            # 局所的な相関計算
            corr = np.correlate(ref_window, fb_window, mode="full")
            lags = np.arange(-len(fb_window) + 1, len(ref_window))
            
            m = (lags >= -max_lag_samples) & (lags <= max_lag_samples)
            corr_filtered = corr[m]
            lags_filtered = lags[m]
            
            if len(corr_filtered) > 0:
                best_lag = -int(lags_filtered[np.argmax(corr_filtered)])
                if 0 <= best_lag <= max_lag_samples:
                    lag_estimates.append(best_lag)
        
        if not lag_estimates:
            return 0.0
        
        # カルマンフィルタで平滑化
        filtered_lag = self.kalman_lag_estimate
        variance = self.kalman_lag_variance
        
        for measurement in lag_estimates:
            # 予測ステップ
            predicted_lag = filtered_lag
            predicted_variance = variance + self.kalman_process_noise
            
            # 更新ステップ
            kalman_gain = predicted_variance / (predicted_variance + self.kalman_measurement_noise)
            filtered_lag = predicted_lag + kalman_gain * (measurement - predicted_lag)
            variance = (1 - kalman_gain) * predicted_variance
        
        # 状態を保存（次回の推定に使用）
        self.kalman_lag_estimate = filtered_lag
        self.kalman_lag_variance = variance
        
        lag_sec = filtered_lag * self.dt_est
        self.get_logger().info(f"Adaptive Kalman lag: {lag_sec:.3f}s (variance: {variance:.3f})")
        return max(0.0, lag_sec)

    def estimate_lag_frequency(self, ref: List[float], fb: List[float], max_lag_s: float) -> float:
        """
        周波数領域での位相差検出
        周期的な動作に特に有効
        """
        ref_arr = np.asarray(ref, dtype=float)
        fb_arr  = np.asarray(fb, dtype=float)

        ref_arr, fb_arr = self._finite_pair(ref_arr, fb_arr)
        if len(ref_arr) < 50:
            return 0.0

        # デトレンド（線形トレンド除去）
        ref_arr = scipy_signal.detrend(ref_arr)
        fb_arr  = scipy_signal.detrend(fb_arr)

        # ゼロパディングでFFTの分解能向上
        n = len(ref_arr)
        n_fft = 2 ** int(np.ceil(np.log2(n * 2)))

        # FFT
        ref_fft = np.fft.rfft(ref_arr, n=n_fft)
        fb_fft  = np.fft.rfft(fb_arr, n=n_fft)

        # クロススペクトル
        cross_spectrum = ref_fft * np.conj(fb_fft)
        
        # 位相差を計算
        phase_diff = np.angle(cross_spectrum)
        
        # 周波数軸
        freqs = np.fft.rfftfreq(n_fft, d=self.dt_est)
        
        # パワーが大きい周波数での位相差を重視
        power = np.abs(cross_spectrum)
        
        # DC成分とナイキスト周波数を除外
        valid_idx = (freqs > 0.01) & (freqs < 1.0 / (2 * self.dt_est) * 0.9) & (power > np.percentile(power, 50))
        
        if np.sum(valid_idx) < 5:
            self.get_logger().warn("Not enough frequency components for lag estimation")
            return 0.0
        
        # 重み付き平均で遅れを計算
        # phase_diff = -2 * pi * freq * lag
        # lag = -phase_diff / (2 * pi * freq)
        
        lags = []
        weights = []
        
        for i in np.where(valid_idx)[0]:
            if freqs[i] > 0:
                lag_at_freq = -phase_diff[i] / (2 * np.pi * freqs[i])
                
                # 位相の折り返しを考慮（複数の候補から最も妥当なものを選択）
                candidates = []
                for k in range(-2, 3):  # ±2周期分の候補
                    candidate = lag_at_freq + k / freqs[i]
                    if 0 <= candidate <= max_lag_s:
                        candidates.append(candidate)
                
                if candidates:
                    # 最も中央値に近いものを選択
                    best_candidate = min(candidates, key=lambda x: abs(x - max_lag_s / 2))
                    lags.append(best_candidate)
                    weights.append(power[i])
        
        if not lags:
            return 0.0
        
        # 重み付き中央値
        lags = np.array(lags)
        weights = np.array(weights)
        
        # 外れ値除去（IQR法）
        q1, q3 = np.percentile(lags, [25, 75])
        iqr = q3 - q1
        mask = (lags >= q1 - 1.5 * iqr) & (lags <= q3 + 1.5 * iqr)
        
        if np.sum(mask) > 0:
            lags = lags[mask]
            weights = weights[mask]
        
        # 重み付き平均
        lag_sec = np.average(lags, weights=weights)
        
        self.get_logger().info(f"Frequency-based lag: {lag_sec:.3f}s (from {len(lags)} freq components)")
        return max(0.0, min(lag_sec, max_lag_s))

    def estimate_lag_polynomial(self, ref: List[float], fb: List[float], max_lag_s: float) -> float:
        """
        多項式フィッティング + 時間微分マッチング
        ノイズに強い速度マッチング
        """
        ref_arr = np.asarray(ref, dtype=float)
        fb_arr  = np.asarray(fb, dtype=float)

        ref_arr, fb_arr = self._finite_pair(ref_arr, fb_arr)
        if len(ref_arr) < 50:
            return 0.0

        # Savitzky-Golayフィルタで平滑化と微分を同時に実施
        window_length = min(51, len(ref_arr) // 3)
        if window_length % 2 == 0:
            window_length -= 1
        if window_length < 5:
            return 0.0
        
        polyorder = 3
        
        try:
            # 位置の平滑化
            ref_smooth = scipy_signal.savgol_filter(ref_arr, window_length, polyorder)
            fb_smooth  = scipy_signal.savgol_filter(fb_arr, window_length, polyorder)
            
            # 速度（1階微分）
            ref_vel = scipy_signal.savgol_filter(ref_arr, window_length, polyorder, deriv=1, delta=self.dt_est)
            fb_vel  = scipy_signal.savgol_filter(fb_arr, window_length, polyorder, deriv=1, delta=self.dt_est)
            
            # 加速度（2階微分）
            ref_acc = scipy_signal.savgol_filter(ref_arr, window_length, polyorder, deriv=2, delta=self.dt_est)
            fb_acc  = scipy_signal.savgol_filter(fb_arr, window_length, polyorder, deriv=2, delta=self.dt_est)
            
        except Exception as e:
            self.get_logger().warn(f"Savitzky-Golay filter failed: {e}")
            return 0.0

        max_lag_samples = int(max_lag_s / self.dt_est)

        # 3つの信号（位置、速度、加速度）で相関を計算して統合
        lags_list = []
        weights_list = []
        
        for signal_name, ref_sig, fb_sig in [
            ("position", ref_smooth, fb_smooth),
            ("velocity", ref_vel, fb_vel),
            ("acceleration", ref_acc, fb_acc)
        ]:
            # 正規化
            ref_norm = (ref_sig - np.mean(ref_sig)) / (np.std(ref_sig) + 1e-9)
            fb_norm  = (fb_sig - np.mean(fb_sig)) / (np.std(fb_sig) + 1e-9)
            
            # 相関計算
            corr = np.correlate(ref_norm, fb_norm, mode="full")
            lags = np.arange(-len(fb_norm) + 1, len(ref_norm))
            
            m = (lags >= -max_lag_samples) & (lags <= max_lag_samples)
            corr_filtered = corr[m]
            lags_filtered = lags[m]
            
            if len(corr_filtered) > 0:
                best_lag = -int(lags_filtered[np.argmax(corr_filtered)])
                correlation_strength = np.max(corr_filtered) / len(ref_norm)
                
                if 0 <= best_lag <= max_lag_samples and correlation_strength > 0.1:
                    lags_list.append(best_lag)
                    # 速度と加速度の方が位相ずれを捉えやすいので重みを増やす
                    if signal_name == "velocity":
                        weights_list.append(correlation_strength * 2.0)
                    elif signal_name == "acceleration":
                        weights_list.append(correlation_strength * 1.5)
                    else:
                        weights_list.append(correlation_strength)
        
        if not lags_list:
            return 0.0
        
        # 重み付き平均
        lags_arr = np.array(lags_list)
        weights_arr = np.array(weights_list)
        
        lag_samples = np.average(lags_arr, weights=weights_arr)
        lag_sec = lag_samples * self.dt_est
        
        self.get_logger().info(f"Polynomial lag: {lag_sec:.3f}s (from {len(lags_list)} signals)")
        return max(0.0, lag_sec)

    def compute_compensated_error(self, ref: List[float], fb: List[float], max_lag_s: float) -> Tuple[List[float], float, List[float]]:
        """
        遅れ補正済み誤差を計算
        
        Returns:
            (compensated_error, lag_sec, fb_shifted)
        """
        lag_method = str(self.get_parameter("lag_method").value)
        
        if lag_method == "dtw":
            lag_sec = self.estimate_lag_dtw(ref, fb, max_lag_s)
        elif lag_method == "gradient":
            lag_sec = self.estimate_lag_gradient(ref, fb, max_lag_s)
        elif lag_method == "adaptive_kalman":
            lag_sec = self.estimate_lag_adaptive_kalman(ref, fb, max_lag_s)
        elif lag_method == "frequency":
            lag_sec = self.estimate_lag_frequency(ref, fb, max_lag_s)
        elif lag_method == "polynomial":
            lag_sec = self.estimate_lag_polynomial(ref, fb, max_lag_s)
        else:  # "correlation"
            lag_samples = self.estimate_lag_samples(ref, fb, max_lag_s)
            lag_sec = lag_samples * self.dt_est

        # 遅れ補正: fbを時間シフト（補間使用）
        n = len(self.t)
        if lag_sec <= 0 or len(ref) != n or len(fb) != n:
            return [ref[i] - fb[i] if i < len(ref) and i < len(fb) else math.nan for i in range(n)], lag_sec, list(fb)

        # 補間を使ってfbを時間シフト
        t_arr = np.array(self.t)
        fb_arr = np.array(fb)
        ref_arr = np.array(ref)
        
        # 有効なデータのみ使用
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
        except Exception as e:
            self.get_logger().warn(f"Interpolation failed: {e}")
            compensated_err = [math.nan] * n
            fb_shifted_list = [math.nan] * n
        
        return compensated_err, lag_sec, fb_shifted_list

    # ---------------------------
    # Plot & CSV
    # ---------------------------

    def make_final_plot(self):
        n_joint_rows = max(1, len(self.plot_joints))

        add_ee_pos = (len(self.ee_ref[0]) == len(self.t) and len(self.t) > 0)
        add_ee_dist = (len(self.ee_dist_err) == len(self.t) and len(self.t) > 0)

        rows = n_joint_rows + (3 if add_ee_pos else 0) + (1 if add_ee_dist else 0)
        fig, axs = plt.subplots(rows, 2, sharex=True, squeeze=False, figsize=(11, 2.2 * rows))

        lag_method = str(self.get_parameter("lag_method").value)

        # --- joints ---
        for r, j in enumerate(self.plot_joints):
            axp = axs[r][0]
            axe = axs[r][1]

            axp.plot(self.t, self.ref[j], color="blue", linestyle="-", label="reference", linewidth=1.5)
            axp.plot(self.t, self.fb[j],  color="green", linestyle="--", label="feedback", linewidth=1.5, alpha=0.7)
            
            # ===== planの軌道をプロット =====
            if self.plan_t and self.plan_pos and j < len(self.plan_pos[0]):
                plan_joint_pos = [pos[j] for pos in self.plan_pos if j < len(pos)]
                axp.plot(self.plan_t, plan_joint_pos, 'o', color="orange", 
                         markersize=3, label="plan", alpha=0.7)

            # ===== 時間シフトされたfeedbackをプロット =====
            comp_err, lag_sec, fb_shifted = self.compute_compensated_error(self.ref[j], self.fb[j], max_lag_s=self.max_lag_s)
            axp.plot(self.t, fb_shifted, color="cyan", linestyle=":", label=f"fb_shifted (+{lag_sec:.3f}s)", linewidth=2, alpha=0.8)

            axe.plot(self.t, self.err[j], color="red", linestyle="-", label="error", linewidth=1.5)
            axe.plot(self.t, comp_err, color="purple", linestyle="--",
                     label=f"compensated ({lag_method})", linewidth=1.5)

            # 最大誤差を計算して表示
            max_err = max_abs(self.err[j])
            max_comp_err = max_abs(comp_err)
            joint_name = self.urdf_joint_names[j] if j < len(self.urdf_joint_names) else f"j{j}"
            
            axp.set_ylabel(f"{joint_name} pos")
            axe.set_ylabel(f"{joint_name} err")
            axp.grid(True)
            axe.grid(True)
            
            # 最大誤差をグラフ上部に表示
            axe.text(0.02, 0.98, f"Max err: {max_err:.6f}\nMax comp-err: {max_comp_err:.6f}\nLag: {lag_sec:.3f}s", 
                     transform=axe.transAxes, verticalalignment='top',
                     bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5),
                     fontsize=8)

            if r == 0:
                axp.legend(loc="upper right", fontsize=8)
                axe.legend(loc="upper right", fontsize=8)

        r0 = n_joint_rows

        # --- end-effector position XYZ ---
        if add_ee_pos:
            labels = ["ee_x (m)", "ee_y (m)", "ee_z (m)"]
            add_ee_plan = (len(self.ee_plan[0]) == len(self.plan_t) and len(self.plan_t) > 0)
            
            for i in range(3):
                r = r0 + i
                axp = axs[r][0]
                axe = axs[r][1]

                axp.plot(self.t, self.ee_ref[i], color="blue", linestyle="-", label="ee_reference", linewidth=1.5)
                axp.plot(self.t, self.ee_fb[i],  color="green", linestyle="--", label="ee_feedback", linewidth=1.5, alpha=0.7)
                
                # ===== planのEE軌道をプロット =====
                if add_ee_plan:
                    axp.plot(self.plan_t, self.ee_plan[i], 'o', color="orange", 
                             markersize=3, label="ee_plan", alpha=0.7)
                
                # ===== 時間シフトされたEE feedbackをプロット =====
                comp_err, lag_sec, ee_fb_shifted = self.compute_compensated_error(self.ee_ref[i], self.ee_fb[i], max_lag_s=self.max_lag_s)
                axp.plot(self.t, ee_fb_shifted, color="cyan", linestyle=":", label=f"ee_fb_shifted (+{lag_sec:.3f}s)", linewidth=2, alpha=0.8)
                
                axe.plot(self.t, self.ee_err[i], color="red", linestyle="-", label="ee_error", linewidth=1.5)
                axe.plot(self.t, comp_err, color="purple", linestyle="--",
                         label=f"ee_compensated", linewidth=1.5)

                # 最大誤差を計算して表示
                max_ee_err = max_abs(self.ee_err[i])
                max_ee_comp_err = max_abs(comp_err)

                axp.set_ylabel(labels[i])
                axe.set_ylabel(labels[i].replace("(m)", "err (m)"))
                axp.grid(True)
                axe.grid(True)
                
                # 最大誤差をグラフ上部に表示
                axe.text(0.02, 0.98, f"Max err: {max_ee_err:.6f} m\nMax comp-err: {max_ee_comp_err:.6f} m\nLag: {lag_sec:.3f}s", 
                         transform=axe.transAxes, verticalalignment='top',
                         bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.5),
                         fontsize=8)

                if r == r0:  # 最初のEEプロットのみlegendを表示
                    axp.legend(loc="upper right", fontsize=8)
                    axe.legend(loc="upper right", fontsize=8)

            r0 += 3

        # --- end-effector 3D distance error ---
        if add_ee_dist:
            r = r0
            axp = axs[r][0]
            axe = axs[r][1]

            # 左側：3D距離誤差のグラフ
            axe.plot(self.t, self.ee_dist_err, color="red", linestyle="-", label="3D distance error", linewidth=1.5)
            
            max_dist_err = max_abs(self.ee_dist_err)
            
            axe.set_ylabel("ee 3D dist err (m)")
            axe.grid(True)
            axe.text(0.02, 0.98, f"Max 3D err: {max_dist_err:.6f} m", 
                     transform=axe.transAxes, verticalalignment='top',
                     bbox=dict(boxstyle='round', facecolor='lightcoral', alpha=0.5),
                     fontsize=8)
            axe.legend(loc="upper right", fontsize=8)
            
            # 右側：空欄（または統計情報など）
            axp.axis('off')  # 右側は非表示

        axs[-1][0].set_xlabel("time (s)")
        axs[-1][1].set_xlabel("time (s)")

        title = f"{self.topic} ({self.field_label or 'unknown'}) samples={len(self.t)} max_lag_s={self.max_lag_s} method={lag_method}"
        fig.suptitle(title)
        fig.tight_layout()
        return fig

    def save_csv(self, path: str):
        add_ee_pos = (len(self.ee_ref[0]) == len(self.t) and len(self.t) > 0)
        add_ee_rpy = (len(self.ee_rpy_ref[0]) == len(self.t) and len(self.t) > 0)
        add_ee_quat = (len(self.ee_quat_ref[0]) == len(self.t) and len(self.t) > 0)

        with open(path, "w", encoding="utf-8") as f:
            header = ["t"]
            
            # URDFの実際の関節名を使用（j0, j1...の代わりに）
            for j in self.plot_joints:
                joint_name = self.urdf_joint_names[j] if j < len(self.urdf_joint_names) else f"j{j}"
                header += [f"{joint_name}_ref", f"{joint_name}_fb", f"{joint_name}_err", f"{joint_name}_vel"]

            if add_ee_pos:
                header += ["ee_ref_x", "ee_ref_y", "ee_ref_z",
                           "ee_fb_x",  "ee_fb_y",  "ee_fb_z",
                           "ee_err_x", "ee_err_y", "ee_err_z",
                           "ee_dist_err"]

            if add_ee_rpy:
                header += ["ee_rpy_ref_roll", "ee_rpy_ref_pitch", "ee_rpy_ref_yaw",
                           "ee_rpy_fb_roll",  "ee_rpy_fb_pitch",  "ee_rpy_fb_yaw",
                           "ee_rpy_err_roll", "ee_rpy_err_pitch", "ee_rpy_err_yaw"]

            if add_ee_quat:
                header += ["ee_quat_ref_x", "ee_quat_ref_y", "ee_quat_ref_z", "ee_quat_ref_w",
                           "ee_quat_fb_x",  "ee_quat_fb_y",  "ee_quat_fb_z",  "ee_quat_fb_w"]

            f.write(",".join(header) + "\n")

            for i, t in enumerate(self.t):
                row = [f"{t:.9f}"]
                for j in self.plot_joints:
                    row.append(f"{self.ref[j][i]}")
                    row.append(f"{self.fb[j][i]}")
                    row.append(f"{self.err[j][i]}")
                    row.append(f"{self.vel[j][i]}")

                if add_ee_pos:
                    row += [f"{self.ee_ref[0][i]}", f"{self.ee_ref[1][i]}", f"{self.ee_ref[2][i]}",
                            f"{self.ee_fb[0][i]}",  f"{self.ee_fb[1][i]}",  f"{self.ee_fb[2][i]}",
                            f"{self.ee_err[0][i]}", f"{self.ee_err[1][i]}", f"{self.ee_err[2][i]}",
                            f"{self.ee_dist_err[i]}"]

                if add_ee_rpy:
                    row += [f"{self.ee_rpy_ref[0][i]}", f"{self.ee_rpy_ref[1][i]}", f"{self.ee_rpy_ref[2][i]}",
                            f"{self.ee_rpy_fb[0][i]}",  f"{self.ee_rpy_fb[1][i]}",  f"{self.ee_rpy_fb[2][i]}",
                            f"{self.ee_rpy_err[0][i]}", f"{self.ee_rpy_err[1][i]}", f"{self.ee_rpy_err[2][i]}"]

                if add_ee_quat:
                    row += [f"{self.ee_quat_ref[0][i]}", f"{self.ee_quat_ref[1][i]}", f"{self.ee_quat_ref[2][i]}", f"{self.ee_quat_ref[3][i]}",
                            f"{self.ee_quat_fb[0][i]}",  f"{self.ee_quat_fb[1][i]}",  f"{self.ee_quat_fb[2][i]}",  f"{self.ee_quat_fb[3][i]}"]

                f.write(",".join(row) + "\n")

    def save_plan_csv(self, path: str):
        """Plan軌道専用のCSVを保存（時刻、関節角度、EE位置のみ）"""
        if not self.plan_t or not self.plan_pos:
            self.get_logger().warn("No plan data to save")
            return
        
        with open(path, "w", encoding="utf-8") as f:
            header = ["t"]
            
            # 関節名
            for j in self.plot_joints:
                joint_name = self.urdf_joint_names[j] if j < len(self.urdf_joint_names) else f"j{j}"
                header.append(joint_name)
            
            # EE位置
            if len(self.ee_plan[0]) > 0:
                header += ["ee_x", "ee_y", "ee_z"]
            
            f.write(",".join(header) + "\n")
            
            # データ行
            for i, t in enumerate(self.plan_t):
                row = [f"{t:.9f}"]
                
                # 関節角度
                if i < len(self.plan_pos):
                    for j in self.plot_joints:
                        if j < len(self.plan_pos[i]):
                            row.append(f"{self.plan_pos[i][j]}")
                        else:
                            row.append("nan")
                
                # EE位置
                if len(self.ee_plan[0]) > 0 and i < len(self.ee_plan[0]):
                    row.append(f"{self.ee_plan[0][i]}")
                    row.append(f"{self.ee_plan[1][i]}")
                    row.append(f"{self.ee_plan[2][i]}")
                
                f.write(",".join(row) + "\n")
        
        self.get_logger().info(f"Saved plan CSV: {path}")

    def _finalize_and_save(self) -> Tuple[bool, str]:
        if len(self.t) == 0:
            return False, "No samples recorded; nothing saved."

        fig = self.make_final_plot()
        fig.savefig(self.out_png, dpi=150)
        self.save_csv(self.out_csv)
        self.save_plan_csv(self.out_plan_csv)

        return True, f"Saved: {self.out_png}, {self.out_csv}, {self.out_plan}, {self.out_plan_csv}, bag={self.bag_dir}"

    # ---------------------------
    # ros2 bag record -a
    # ---------------------------

    def _start_bag_record_all(self):
        # os.makedirs(self.bag_dir, exist_ok=True)

        cmd = ["ros2", "bag", "record", "-a", "-o", self.bag_dir]
        self.get_logger().info("Starting bag: " + " ".join(cmd))

        bag_log_path = os.path.join(self.out_dir, "bag_record.log")
        self._bag_log = open(bag_log_path, "w", encoding="utf-8")

        self._bag_proc = subprocess.Popen(
            cmd,
            stdout=self._bag_log,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,   # ★
        )

    def _stop_bag(self):
        if self._bag_proc is None:
            return

        self.get_logger().info("Stopping bag (SIGINT)...")
        try:
            self._bag_proc.send_signal(signal.SIGINT)
            self._bag_proc.wait(timeout=15.0)
        except Exception:
            self.get_logger().warn("Bag did not stop gracefully; killing.")
            try:
                self._bag_proc.kill()
            except Exception:
                pass
        finally:
            self._bag_proc = None


def main():
    rclpy.init()
    node = TrajFollowRecordActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()