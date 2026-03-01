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

        self.declare_parameter("state_topic", "/zx200/upper_arm_controller/controller_state")
        self.declare_parameter("output_root", "/home/common/3_SIP/tms_ws/src/traj_follow_measurement/data")
        self.declare_parameter("record_bag_all", True)     # -a 相当をデフォルトで回すか

        self.declare_parameter("max_lag_s", 2.0)
        self.declare_parameter("phase_use_velocity", False)

        # ===== FK/URDF 追加パラメータ =====
        self.declare_parameter("urdf_path", "/home/common/3_SIP/tms_ws/src/traj_follow_measurement/traj_follow_plotter/urdf/zx200.urdf")
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

        # URDF 読み込み & FK チェーン準備（失敗しても計測自体は続行）
        self._init_fk_from_urdf()

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

        # plan（Goal内容）を即保存
        self._save_plan_yaml(goal_handle.request)

        # state購読開始
        self._reset_buffers()
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
            "plan": self._to_plain([message_to_ordereddict(rt) for rt in goal_msg.plan]),
        }
        with open(self.out_plan, "w", encoding="utf-8") as f:
            yaml.safe_dump(data, f, sort_keys=False, allow_unicode=True)
        self.get_logger().info(f"Saved plan: {self.out_plan}")

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

                for k, jn in enumerate(self._chain_joint_names):  # enumerate を追加
                    idx = self._joint_name_to_msg_index[jn]
                    q_ref[k] = ref_pos[idx] if idx < len(ref_pos) else float("nan")
                    q_fb[k]  = fb_pos[idx]  if idx < len(fb_pos)  else float("nan")

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

            except Exception as e:
                self.get_logger().warn("FK failed during run; disabling FK.\n" + traceback.format_exc())
                self._fk_ready = False

    # ---------------------------
    # Phase lag (optional)
    # ---------------------------

    def _finite_pair(self, a: np.ndarray, b: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        ok = np.isfinite(a) & np.isfinite(b)
        return a[ok], b[ok]

    def estimate_lag_samples(self, ref: List[float], fb: List[float], max_lag_s: float) -> int:
        ref_arr = np.asarray(ref, dtype=float)
        fb_arr  = np.asarray(fb, dtype=float)

        ref_arr, fb_arr = self._finite_pair(ref_arr, fb_arr)
        if len(ref_arr) < 20:
            return 0

        if self.phase_use_velocity:
            ref_arr = np.diff(ref_arr)
            fb_arr  = np.diff(fb_arr)

        if len(ref_arr) < 20:
            return 0

        ref_arr = ref_arr - np.mean(ref_arr)
        fb_arr  = fb_arr - np.mean(fb_arr)

        max_lag = int(max(0.0, max_lag_s) / self.dt_est)
        if max_lag <= 0:
            return 0

        corr = np.correlate(ref_arr, fb_arr, mode="full")
        lags = np.arange(-len(fb_arr) + 1, len(ref_arr))

        m = (lags >= -max_lag) & (lags <= max_lag)
        corr = corr[m]
        lags = lags[m]
        if len(corr) == 0:
            return 0

        best_lag = int(lags[np.argmax(corr)])
        lag_samples = -best_lag
        if lag_samples < 0:
            lag_samples = 0
        return int(lag_samples)

    def phase_shift_error(self, ref: List[float], fb: List[float], lag_samples: int) -> List[float]:
        n = min(len(ref), len(fb))
        out = [math.nan] * n
        for i in range(n):
            j = i - lag_samples
            if 0 <= j < n:
                out[i] = ref[j] - fb[i]
        return out

    # ---------------------------
    # Plot & CSV
    # ---------------------------

    def make_final_plot(self):
        n_joint_rows = max(1, len(self.plot_joints))

        add_ee_pos = (len(self.ee_ref[0]) == len(self.t) and len(self.t) > 0)
        add_ee_rpy = (len(self.ee_rpy_ref[0]) == len(self.t) and len(self.t) > 0)

        rows = n_joint_rows + (3 if add_ee_pos else 0) + (3 if add_ee_rpy else 0)
        fig, axs = plt.subplots(rows, 2, sharex=True, squeeze=False, figsize=(11, 2.2 * rows))

        # --- joints ---
        for r, j in enumerate(self.plot_joints):
            axp = axs[r][0]
            axe = axs[r][1]

            axp.plot(self.t, self.ref[j], color="blue", linestyle="-", label="reference")
            axp.plot(self.t, self.fb[j],  color="green", linestyle="--", label="feedback")

            axe.plot(self.t, self.err[j], color="red", linestyle="-", label="error")

            lag = self.estimate_lag_samples(self.ref[j], self.fb[j], max_lag_s=self.max_lag_s)
            ph_err = self.phase_shift_error(self.ref[j], self.fb[j], lag_samples=lag)
            axe.plot(self.t, ph_err, color="purple", linestyle="--",
                     label=f"phase-error (lag={lag} samples)")

            axp.set_ylabel(f"j{j} pos")
            axe.set_ylabel(f"j{j} err")
            axp.grid(True)
            axe.grid(True)

            if r == 0:
                axp.legend(loc="upper right")
                axe.legend(loc="upper right")

        r0 = n_joint_rows

        # --- end-effector position XYZ ---
        if add_ee_pos:
            labels = ["ee_x (m)", "ee_y (m)", "ee_z (m)"]
            for i in range(3):
                r = r0 + i
                axp = axs[r][0]
                axe = axs[r][1]

                axp.plot(self.t, self.ee_ref[i], color="blue", linestyle="-", label="ee_reference")
                axp.plot(self.t, self.ee_fb[i],  color="green", linestyle="--", label="ee_feedback")
                axe.plot(self.t, self.ee_err[i], color="red", linestyle="-", label="ee_error")

                lag = self.estimate_lag_samples(self.ee_ref[i], self.ee_fb[i], max_lag_s=self.max_lag_s)
                ph_err = self.phase_shift_error(self.ee_ref[i], self.ee_fb[i], lag_samples=lag)
                axe.plot(self.t, ph_err, color="purple", linestyle="--",
                         label=f"ee_phase-error (lag={lag} samples)")

                axp.set_ylabel(labels[i])
                axe.set_ylabel(labels[i].replace("(m)", "err (m)"))
                axp.grid(True)
                axe.grid(True)

                if r == 0:
                    axp.legend(loc="upper right")
                    axe.legend(loc="upper right")

            r0 += 3

        # --- end-effector orientation RPY ---
        if add_ee_rpy:
            labels = ["ee_roll (rad)", "ee_pitch (rad)", "ee_yaw (rad)"]
            for i in range(3):
                r = r0 + i
                axp = axs[r][0]
                axe = axs[r][1]

                axp.plot(self.t, self.ee_rpy_ref[i], color="blue", linestyle="-", label="ee_rpy_reference")
                axp.plot(self.t, self.ee_rpy_fb[i],  color="green", linestyle="--", label="ee_rpy_feedback")
                axe.plot(self.t, self.ee_rpy_err[i], color="red", linestyle="-", label="ee_rpy_error")

                lag = self.estimate_lag_samples(self.ee_rpy_ref[i], self.ee_rpy_fb[i], max_lag_s=self.max_lag_s)
                ph_err = self.phase_shift_error(self.ee_rpy_ref[i], self.ee_rpy_fb[i], lag_samples=lag)
                axe.plot(self.t, ph_err, color="purple", linestyle="--",
                         label=f"ee_rpy_phase-error (lag={lag} samples)")

                axp.set_ylabel(labels[i])
                axe.set_ylabel(labels[i].replace("(rad)", "err (rad)"))
                axp.grid(True)
                axe.grid(True)

                if r == 0:
                    axp.legend(loc="upper right")
                    axe.legend(loc="upper right")

        axs[-1][0].set_xlabel("time (s)")
        axs[-1][1].set_xlabel("time (s)")

        title = f"{self.topic} ({self.field_label or 'unknown'}) samples={len(self.t)} max_lag_s={self.max_lag_s}"
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
                           "ee_err_x", "ee_err_y", "ee_err_z"]

            if add_ee_rpy:
                header += ["ee_rpy_ref_roll", "ee_rpy_ref_pitch", "ee_rpy_ref_yaw",
                           "ee_rpy_fb_roll",  "ee_rpy_fb_pitch",  "ee_rpy_fb_yaw",
                           "ee_rpy_err_roll", "ee_rpy_err_pitch", "ee_rpy_err_yaw"]

            # Quaternion は "差" の扱いが難しいので（符号反転同値など）、ref/fb のみ保存
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
                            f"{self.ee_err[0][i]}", f"{self.ee_err[1][i]}", f"{self.ee_err[2][i]}"]

                if add_ee_rpy:
                    row += [f"{self.ee_rpy_ref[0][i]}", f"{self.ee_rpy_ref[1][i]}", f"{self.ee_rpy_ref[2][i]}",
                            f"{self.ee_rpy_fb[0][i]}",  f"{self.ee_rpy_fb[1][i]}",  f"{self.ee_rpy_fb[2][i]}",
                            f"{self.ee_rpy_err[0][i]}", f"{self.ee_rpy_err[1][i]}", f"{self.ee_rpy_err[2][i]}"]

                if add_ee_quat:
                    row += [f"{self.ee_quat_ref[0][i]}", f"{self.ee_quat_ref[1][i]}", f"{self.ee_quat_ref[2][i]}", f"{self.ee_quat_ref[3][i]}",
                            f"{self.ee_quat_fb[0][i]}",  f"{self.ee_quat_fb[1][i]}",  f"{self.ee_quat_fb[2][i]}",  f"{self.ee_quat_fb[3][i]}"]

                f.write(",".join(row) + "\n")

    def _finalize_and_save(self) -> Tuple[bool, str]:
        if len(self.t) == 0:
            return False, "No samples recorded; nothing saved."

        fig = self.make_final_plot()
        fig.savefig(self.out_png, dpi=150)
        self.save_csv(self.out_csv)

        return True, f"Saved: {self.out_png}, {self.out_csv}, {self.out_plan}, bag={self.bag_dir}"

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