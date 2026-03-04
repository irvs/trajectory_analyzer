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

# ===== 共通モジュールをインポート =====
from .trajectory_analyzer import (
    TrajectoryAnalyzer,
    max_abs,
    quat_to_rpy
)


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
        self.declare_parameter("record_bag_all", True)     # -a相当をデフォルトで回すか

        self.declare_parameter("max_lag_s", 5.0)
        self.declare_parameter("phase_use_velocity", False)
        self.declare_parameter("lag_method", "frequency")  # "correlation", "dtw", "gradient", "adaptive_kalman", "frequency", "polynomial"

        # ===== FK/URDF 追加パラメータ =====
        self.declare_parameter("urdf_path", os.path.normpath(default_urdf))  # 相対パス: traj_follow_plotterから見て../urdf/
        self.declare_parameter("fk_base_link", "base_link")
        self.declare_parameter("fk_tip_link", "bucket_end_link")

        # ===== 共通解析器を作成 =====
        self.analyzer = TrajectoryAnalyzer(
            max_lag_s=float(self.get_parameter("max_lag_s").value),
            lag_method=str(self.get_parameter("lag_method").value),
            phase_use_velocity=bool(self.get_parameter("phase_use_velocity").value)
        )

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
            "time_scaling": [float(x) for x in goal_msg.time_scaling],
            "velocity_scaling": [float(x) for x in goal_msg.velocity_scaling],
            "acceleration_scaling": [float(x) for x in goal_msg.acceleration_scaling],
            "plan": self._to_plain(message_to_ordereddict(goal_msg.plan)),
        }
        with open(self.out_plan, "w", encoding="utf-8") as f:
            yaml.safe_dump(data, f, sort_keys=False, allow_unicode=True)
        self.get_logger().info(f"Saved plan: {self.out_plan}")
        
        # スケーリング値をログ出力とフォルダ名に含める
        time_scale = data['time_scaling'][0] if data['time_scaling'] else 1.0
        vel_scale = data['velocity_scaling'][0] if data['velocity_scaling'] else 0.0
        acc_scale = data['acceleration_scaling'][0] if data['acceleration_scaling'] else 0.0
        
        self.get_logger().info(f"Scaling values - time: {time_scale:.3f}, velocity: {vel_scale:.3f}, accel: {acc_scale:.3f}")
        
        # フォルダ名にスケーリング値を追加してリネーム
        old_dir = self.out_dir
        parent_dir = os.path.dirname(old_dir)
        base_name = os.path.basename(old_dir)
        
        # run_YYYYMMDD_HHMMSS_tXXX_vXXX_aXXX の形式
        new_dir = os.path.join(parent_dir, f"{base_name}_t{time_scale:.3f}_v{vel_scale:.3f}_a{acc_scale:.3f}")
        
        try:
            os.rename(old_dir, new_dir)
            self.out_dir = new_dir
            
            # 全てのパスを更新
            self.out_png = os.path.join(self.out_dir, "plot.png")
            self.out_csv = os.path.join(self.out_dir, "data.csv")
            self.out_plan = os.path.join(self.out_dir, "plan.yaml")
            self.out_plan_csv = os.path.join(self.out_dir, "plan.csv")
            self.bag_dir = os.path.join(self.out_dir, "bag")
            
            self.get_logger().info(f"Renamed output dir to: {self.out_dir}")
        except Exception as e:
            self.get_logger().warn(f"Failed to rename directory with scaling values: {e}")
        
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
    # Phase lag estimation は trajectory_analyzer.py の TrajectoryAnalyzer を使用
    # ---------------------------

    # ---------------------------
    # Plot & CSV
    # ---------------------------

    def save_csv(self, path: str):
        add_ee_pos = (len(self.ee_ref[0]) == len(self.t) and len(self.t) > 0)
        add_ee_rpy = (len(self.ee_rpy_ref[0]) == len(self.t) and len(self.t) > 0)
        add_ee_quat = (len(self.ee_rpy_ref[0]) == len(self.t) and len(self.t) > 0)

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

        # ===== 共通モジュールを使ってプロット作成 =====
        # データを辞書形式に変換
        data = {
            't': self.t,
            'dt_est': self.dt_est,
            'joints': {},
            'ee': {},
            'ee_dist_err': self.ee_dist_err
        }
        
        for j in self.plot_joints:
            joint_name = self.urdf_joint_names[j] if j < len(self.urdf_joint_names) else f"j{j}"
            data['joints'][joint_name] = {
                'ref': self.ref[j],
                'fb': self.fb[j],
                'err': self.err[j],
                'vel': self.vel[j]
            }
        
        if len(self.ee_ref[0]) > 0:
            data['ee']['pos'] = {
                'ref': self.ee_ref,
                'fb': self.ee_fb,
                'err': self.ee_err
            }
        
        # Planデータ
        plan_data = None
        if self.plan_t and self.plan_pos:
            plan_data = {
                't': self.plan_t,
                'joints': {},
                'ee': self.ee_plan if len(self.ee_plan[0]) > 0 else None
            }
            
            for j in self.plot_joints:
                joint_name = self.urdf_joint_names[j] if j < len(self.urdf_joint_names) else f"j{j}"
                if self.plan_pos and j < len(self.plan_pos[0]):
                    plan_data['joints'][joint_name] = [pos[j] for pos in self.plan_pos if j < len(pos)]
        
        # 共通モジュールを使ってプロット作成
        from .trajectory_analyzer import create_plot, save_compensated_csv
        try:
            create_plot(
                data=data,
                plan_data=plan_data,
                analyzer=self.analyzer,
                output_path=self.out_png,
                topic=self.topic,
                field_label=self.field_label or "unknown"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to create plot: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
        
        self.save_csv(self.out_csv)
        self.save_plan_csv(self.out_plan_csv)
        
        # 補正済みCSVを保存
        compensated_csv = os.path.join(self.out_dir, 'data_compensated.csv')
        try:
            save_compensated_csv(data, self.analyzer, compensated_csv)
            self.get_logger().info(f"Saved compensated CSV: {compensated_csv}")
        except Exception as e:
            self.get_logger().error(f"Failed to save compensated CSV: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())

        return True, f"Saved: {self.out_png}, {self.out_csv}, {compensated_csv}, {self.out_plan}, {self.out_plan_csv}, bag={self.bag_dir}"

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