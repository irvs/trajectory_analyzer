#!/usr/bin/env python3
import os
import math
import signal
import subprocess
import datetime
import time
import tempfile
from typing import Optional, List, Tuple, Dict
from collections import OrderedDict
import threading

import numpy as np
from scipy.interpolate import interp1d
from scipy import signal as scipy_signal
from scipy.optimize import minimize_scalar

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState

# planの保存用（YAML化）
from rosidl_runtime_py.convert import message_to_ordereddict
import yaml

import matplotlib.pyplot as plt

import asyncio

# from traj_recorder_msgs.action import TrajFollow
from tms_msg_ts.action import AnalyzeTrajectory

# ===== 共通モジュールをインポート =====
from .trajectory_analyzer import (
    TrajectoryAnalyzer,
    FKSolver
)

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


class RecordingContext:
    """
    1つのアクションゴール（計測）に関するデータと状態を保持するクラス
    """
    def __init__(self, goal_id: str, topic: str, out_dir: Optional[str], save_files: bool = True):
        self.goal_id = goal_id
        self.topic = topic
        self.out_dir = out_dir
        # False の場合、フォルダ作成・ファイル保存（bag含む）を一切行わない
        self.save_files = save_files

        self.started = False
        self.start_time: Optional[float] = None
        self.dt_est = 0.02

        # 出力ファイルパス（save_files=False の場合は out_dir が None のため未使用）
        if out_dir is not None:
            self.out_png = os.path.join(out_dir, "plot.png")
            self.out_csv = os.path.join(out_dir, "data.csv")
            self.out_plan = os.path.join(out_dir, "plan.yaml")
            self.out_plan_csv = os.path.join(out_dir, "plan.csv")
            self.bag_dir = os.path.join(out_dir, "bag")
        else:
            self.out_png = None
            self.out_csv = None
            self.out_plan = None
            self.out_plan_csv = None
            self.bag_dir = None

        # バッファ
        self.t: List[float] = []
        self.ref: List[List[float]] = []
        self.fb:  List[List[float]] = []
        self.err: List[List[float]] = []
        self.vel: List[List[float]] = []

        self.ee_ref = [[], [], []]
        self.ee_fb  = [[], [], []]
        self.ee_err = [[], [], []]
        self.ee_dist_err = []

        self.field_label = None
        self.n_all = None
        self.plot_joints: List[int] = []

        # plan軌道
        self.plan_t = []
        self.plan_pos = []
        self.plan_joint_names = []
        self.ee_plan = [[], [], []]

        # FK関連マッピング（runごとに作り直す）
        self._joint_name_to_msg_index = {}

        # bag用
        self.bag_proc: Optional[subprocess.Popen] = None
        self.bag_log = None

    def ensure_buffers(self, n_all: int):
        if self.n_all == n_all and self.ref:
            return
        self.n_all = n_all
        self.plot_joints = list(range(n_all))
        self.ref = [[] for _ in range(n_all)]
        self.fb  = [[] for _ in range(n_all)]
        self.err = [[] for _ in range(n_all)]
        self.vel = [[] for _ in range(n_all)]


class TrajFollowRecordActionServer(Node):
    def __init__(self):
        super().__init__("traj_follow_record_action_server")

        # URDFの実際の関節名を定義（zx200用）
        self.urdf_joint_names = [
            "swing_joint",
            "boom_joint",
            "arm_joint",
            "bucket_joint",
        ]

        from ament_index_python.packages import get_package_share_directory
        pkg_share = get_package_share_directory('traj_follow_plotter')
        default_output = os.path.join(pkg_share, "..", "..", "..", "data")
        default_urdf = os.path.join(pkg_share, "urdf", "zx200.urdf")

        self.declare_parameter("state_topic", "/zx200/upper_arm_controller/controller_state")
        self.declare_parameter("output_root", os.path.normpath(default_output))
        self.declare_parameter("record_bag_all", True)

        # ===== 記録モード =====
        # full       : 通常どおり記録・保存する（デフォルト）
        # no_process : アクションは作成しリクエストは受け付けるが、記録処理は一切行わない
        # no_save    : 状態の収集・FK計算などの処理は行うが、フォルダ作成やファイル保存は行わない
        self.declare_parameter("recording_mode", "full")

        # ===== FK/URDF パラメータ =====
        self.declare_parameter("urdf_path", os.path.normpath(default_urdf))
        self.declare_parameter("fk_base_link", "base_link")
        self.declare_parameter("fk_tip_link", "bucket_end_link")

        # ===== 共通解析器を作成 =====
        self.analyzer = TrajectoryAnalyzer()

        # ---- Callback Groups ----
        self._action_cb_group = ReentrantCallbackGroup()
        self._sub_cb_group = MutuallyExclusiveCallbackGroup()

        # ---- Action Server ----
        self._action_srv = ActionServer(
            self,
            AnalyzeTrajectory,
            "analyze_trajectory",
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
            callback_group=self._action_cb_group,
        )

        # ---- 並列実行管理 ----
        self._active_recordings: Dict[str, RecordingContext] = {}
        self._recording_lock = threading.Lock()
        self._topic_subscriptions: Dict[str, rclpy.subscription.Subscription] = {}

        self.output_root = str(self.get_parameter("output_root").value)
        self._record_bag_all = bool(self.get_parameter("record_bag_all").value)

        # ── マスターセッションディレクトリ（ノード起動中の最初のrunで作成、以降共有） ──
        self._master_session_dir: Optional[str] = None
        self._master_session_dir_lock = threading.Lock()

        # matplotlibは最後だけ描画
        plt.ioff()

        # ===== FK 状態 =====
        self._fk_solver = None
        self._fk_ready = False
        self._fk_failed_reason = ""

        # URDF 読み込み & FK チェーン準備（失敗しても計測自体は続行）
        self._init_fk_from_urdf()

    # ---------------------------
    # FK init
    # ---------------------------

    def _init_fk_from_urdf(self):
        urdf_path = str(self.get_parameter("urdf_path").value)
        base_link = str(self.get_parameter("fk_base_link").value)
        tip_link  = str(self.get_parameter("fk_tip_link").value)

        # 共通モジュールのFKSolverを使用
        self._fk_solver = FKSolver(urdf_path, base_link, tip_link)
        self._fk_ready = self._fk_solver.ready
        
        if self._fk_ready:
            self._chain_joint_names = self._fk_solver.get_joint_names()
            self.get_logger().info(
                f"FK enabled: {base_link} -> {tip_link}, joints={len(self._chain_joint_names)}"
            )
        else:
            self._fk_failed_reason = "FKSolver initialization failed"
            self.get_logger().warn("FK disabled")

    # ---------------------------
    # Action callbacks
    # ---------------------------

    def goal_cb(self, goal_request: AnalyzeTrajectory.Goal):
        self.get_logger().info("goal_cb called")
        # 並行実行を許可するため常にACCEPT
        self.get_logger().info("Goal ACCEPT")
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        return CancelResponse.ACCEPT

    def execute_cb(self, goal_handle):
        goal_id = str(goal_handle.goal_id.uuid)
        self.get_logger().info(f"Executing goal: {goal_id}")

        recording_mode = str(self.get_parameter("recording_mode").value).strip().lower()
        if recording_mode not in ("full", "no_process", "no_save"):
            self.get_logger().warn(
                f"Unknown recording_mode '{recording_mode}'; falling back to 'full'"
            )
            recording_mode = "full"

        if recording_mode == "no_process":
            return self._execute_no_process(goal_handle, goal_id)

        save_files = recording_mode != "no_save"

        # 準備
        topic = str(self.get_parameter("state_topic").value)
        out_dir = self._prepare_output_dir() if save_files else None
        ctx = RecordingContext(goal_id, topic, out_dir, save_files=save_files)

        self._save_plan_yaml(ctx, goal_handle.request)

        # サブスクリプション取得
        with self._recording_lock:
            if topic not in self._topic_subscriptions:
                self.get_logger().info(f"Creating subscription for topic: {topic}")
                sub = self.create_subscription(
                    JointTrajectoryControllerState,
                    topic,
                    lambda msg, t=topic: self.on_state(msg, t),
                    10,
                    callback_group=self._sub_cb_group,
                )
                self._topic_subscriptions[topic] = sub
            
            # アクティブな記録として登録
            self._active_recordings[goal_id] = ctx

        try:
            ctx.started = True
            ctx.start_time = self.get_clock().now().nanoseconds * 1e-9

            if self._record_bag_all and ctx.save_files:
                self._start_bag_record_all(ctx)

            fb_msg = AnalyzeTrajectory.Feedback()
            fb_msg.status = f"recording: {topic}"
            goal_handle.publish_feedback(fb_msg)

            # Cancel されるまで待つ
            while rclpy.ok() and not goal_handle.is_cancel_requested:
                time.sleep(0.05)

            if goal_handle.is_cancel_requested:
                fb_msg.status = "cancel_requested: stopping"
                goal_handle.publish_feedback(fb_msg)
                goal_handle.canceled()
            
            ctx.started = False
            self._stop_bag(ctx)

            fb_msg.status = "saving"
            goal_handle.publish_feedback(fb_msg)

            ok, msg = self._finalize_and_save(ctx)
            
            result = AnalyzeTrajectory.Result()
            result.ok = bool(ok)
            self.get_logger().info(msg)
            return result

        except Exception as e:
            self.get_logger().error(f"Error in execute_cb: {e}\n{traceback.format_exc()}")
            result = AnalyzeTrajectory.Result()
            result.ok = False
            return result
        finally:
            with self._recording_lock:
                if goal_id in self._active_recordings:
                    del self._active_recordings[goal_id]
            self.get_logger().info(f"Finished goal: {goal_id}")

    def _execute_no_process(self, goal_handle, goal_id: str):
        """
        recording_mode='no_process' 用の実行パス。
        アクションのリクエストは受け付けるが、購読・記録・保存を一切行わない。
        """
        self.get_logger().info(
            f"[{goal_id}] recording_mode=no_process: accepting goal without recording"
        )

        fb_msg = AnalyzeTrajectory.Feedback()
        fb_msg.status = "no_process mode: recording disabled"
        goal_handle.publish_feedback(fb_msg)

        while rclpy.ok() and not goal_handle.is_cancel_requested:
            time.sleep(0.05)

        if goal_handle.is_cancel_requested:
            fb_msg.status = "cancel_requested: no-op"
            goal_handle.publish_feedback(fb_msg)
            goal_handle.canceled()

        result = AnalyzeTrajectory.Result()
        result.ok = True
        self.get_logger().info(f"[{goal_id}] no_process mode: nothing recorded or saved")
        return result

    # ---------------------------
    # Output directory & plan save
    # ---------------------------

    def _prepare_output_dir(self):
        """出力ディレクトリを作成。最初のrunでマスターセッションディレクトリも作成。"""
        os.makedirs(self.output_root, exist_ok=True)

        # マスターセッションディレクトリをノード起動後の最初のrun時に作成
        with self._master_session_dir_lock:
            if self._master_session_dir is None:
                session_stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M")
                session_dir = os.path.join(self.output_root, f"traj_record_{session_stamp}")
                os.makedirs(session_dir, exist_ok=True)
                self._master_session_dir = session_dir
                self.get_logger().info(f"Created master session dir: {self._master_session_dir}")
            session_root = self._master_session_dir

        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")  # microsecond追加
        out_dir = os.path.join(session_root, f"run_{stamp}")
        os.makedirs(out_dir, exist_ok=True)
        return out_dir


    def _to_plain(self, obj):
        """OrderedDictなどをYAMLで安全に書ける plain dict/list に変換"""
        if isinstance(obj, OrderedDict):
            return {k: self._to_plain(v) for k, v in obj.items()}
        if isinstance(obj, dict):
            return {k: self._to_plain(v) for k, v in obj.items()}
        if isinstance(obj, (list, tuple)):
            return [self._to_plain(v) for v in obj]
        return obj

    def _save_plan_yaml(self, ctx: RecordingContext, goal_msg: AnalyzeTrajectory.Goal):
        if not ctx.save_files:
            # save_files=False の場合はファイル・フォルダを一切作成せず、
            # メモリ上の処理に必要な軌道データの抽出のみ行う
            self.get_logger().info(
                f"[{ctx.goal_id}] save_files=False: skipping plan yaml save and directory operations"
            )
            self._extract_plan_trajectory(ctx, goal_msg.plan)
            return

        data = {
            "time_scaling": [float(x) for x in goal_msg.time_scaling],
            "velocity_scaling": [float(x) for x in goal_msg.velocity_scaling],
            "acceleration_scaling": [float(x) for x in goal_msg.acceleration_scaling],
            "plan": self._to_plain(message_to_ordereddict(goal_msg.plan)),
        }
        with open(ctx.out_plan, "w", encoding="utf-8") as f:
            yaml.safe_dump(data, f, sort_keys=False, allow_unicode=True)
        self.get_logger().info(f"Saved plan: {ctx.out_plan}")

        # スケーリング値をログ出力とフォルダ名に含める
        time_scale = data['time_scaling'][0] if data['time_scaling'] else 1.0
        vel_scale = data['velocity_scaling'][0] if data['velocity_scaling'] else 0.0
        acc_scale = data['acceleration_scaling'][0] if data['acceleration_scaling'] else 0.0

        self.get_logger().info(f"Scaling values - time: {time_scale:.3f}, velocity: {vel_scale:.3f}, accel: {acc_scale:.3f}")

        # ── マスターセッションディレクトリを最初のrunのみスケーリング値でリネーム ──
        with self._master_session_dir_lock:
            if self._master_session_dir and not os.path.basename(self._master_session_dir).count('_t'):
                # まだスケーリング値が付いていない場合のみリネーム
                old_session = self._master_session_dir
                new_session = f"{old_session}_t{time_scale:.3f}_v{vel_scale:.3f}_a{acc_scale:.3f}"
                try:
                    os.rename(old_session, new_session)
                    self._master_session_dir = new_session
                    self.get_logger().info(f"Renamed master session dir to: {new_session}")
                    # run_* の親ディレクトリも更新
                    ctx.out_dir = ctx.out_dir.replace(old_session, new_session)
                except Exception as e:
                    self.get_logger().warn(f"Failed to rename master session dir: {e}")

        # フォルダ名にスケーリング値を追加してリネーム
        old_dir = ctx.out_dir
        parent_dir = os.path.dirname(old_dir)
        base_name = os.path.basename(old_dir)

        # run_YYYYMMDD_HHMMSS_tXXX_vXXX_aXXX の形式
        new_dir = os.path.join(parent_dir, f"{base_name}_t{time_scale:.3f}_v{vel_scale:.3f}_a{acc_scale:.3f}")

        try:
            os.rename(old_dir, new_dir)
            ctx.out_dir = new_dir

            # 全てのパスを更新
            ctx.out_png = os.path.join(ctx.out_dir, "plot.png")
            ctx.out_csv = os.path.join(ctx.out_dir, "data.csv")
            ctx.out_plan = os.path.join(ctx.out_dir, "plan.yaml")
            ctx.out_plan_csv = os.path.join(ctx.out_dir, "plan.csv")
            ctx.bag_dir = os.path.join(ctx.out_dir, "bag")

            self.get_logger().info(f"Renamed output dir to: {ctx.out_dir}")
        except Exception as e:
            self.get_logger().warn(f"Failed to rename directory with scaling values: {e}")

        # ===== planから軌道データを抽出 =====
        self._extract_plan_trajectory(ctx, goal_msg.plan)

    def _extract_plan_trajectory(self, ctx: RecordingContext, plan_trajectory):
        """planからtime_from_startと関節位置を抽出"""
        ctx.plan_t = []
        ctx.plan_pos = []
        ctx.plan_joint_names = []
        
        try:
            # RobotTrajectory内のjoint_trajectoryを取得
            joint_traj = plan_trajectory.joint_trajectory
            
            # joint_namesを取得（関節の順序を把握）
            joint_names = list(joint_traj.joint_names)
            ctx.plan_joint_names = joint_names
            
            # 各pointから時刻と位置を抽出
            for point in joint_traj.points:
                # time_from_startをfloat秒に変換
                t_sec = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
                ctx.plan_t.append(t_sec)
                
                # 位置データを保存（全関節分）
                positions = list(point.positions)
                ctx.plan_pos.append(positions)
            
            if ctx.plan_t:
                self.get_logger().info(f"Extracted plan trajectory: {len(ctx.plan_t)} points")
                
                # ===== planからEE位置をFKで計算 =====
                self._compute_plan_ee_positions(ctx)
            else:
                self.get_logger().warn("No trajectory points found in plan")
                
        except Exception as e:
            self.get_logger().warn(f"Failed to extract plan trajectory: {e}\n{traceback.format_exc()}")
            ctx.plan_t = []
            ctx.plan_pos = []

    def _compute_plan_ee_positions(self, ctx: RecordingContext):
        """Plan軌道の各点でFKを計算してEE位置を保存"""
        if not self._fk_ready or self._fk_solver is None:
            self.get_logger().warn("FK not ready; skipping plan EE position calculation")
            return
        
        if not ctx.plan_joint_names:
            self.get_logger().warn("Plan joint names not available; skipping plan EE position calculation")
            return
        
        ctx.ee_plan = [[], [], []]  # x, y, z
        
        try:
            # Plan軌道のjoint_namesからchain_joint_namesへのマッピングを作成
            chain_joint_names = self._fk_solver.get_joint_names()
            plan_to_chain_map = {}
            
            for chain_jn in chain_joint_names:
                if chain_jn in ctx.plan_joint_names:
                    plan_to_chain_map[chain_jn] = ctx.plan_joint_names.index(chain_jn)
                else:
                    self.get_logger().warn(f"Chain joint '{chain_jn}' not found in plan joint names")
            
            for positions in ctx.plan_pos:
                # 関節角度を辞書形式で準備
                joint_positions = {}
                for chain_jn in chain_joint_names:
                    if chain_jn in plan_to_chain_map:
                        plan_idx = plan_to_chain_map[chain_jn]
                        if plan_idx < len(positions):
                            joint_positions[chain_jn] = positions[plan_idx]
                        else:
                            joint_positions[chain_jn] = 0.0
                    else:
                        joint_positions[chain_jn] = 0.0
                
                # FK計算
                ee_pos = self._fk_solver.compute(joint_positions)
                
                if ee_pos is not None:
                    ctx.ee_plan[0].append(ee_pos[0])
                    ctx.ee_plan[1].append(ee_pos[1])
                    ctx.ee_plan[2].append(ee_pos[2])
                else:
                    ctx.ee_plan[0].append(math.nan)
                    ctx.ee_plan[1].append(math.nan)
                    ctx.ee_plan[2].append(math.nan)
            
            self.get_logger().info(f"Computed plan EE positions: {len(ctx.ee_plan[0])} points")
            
        except Exception as e:
            self.get_logger().warn(f"Failed to compute plan EE positions: {e}\n{traceback.format_exc()}")
            ctx.ee_plan = [[], [], []]

    def on_state(self, msg: JointTrajectoryControllerState, topic: str):
        now = self.get_clock().now().nanoseconds * 1e-9

        # アクティブなすべての記録に対してデータを追加
        with self._recording_lock:
            active_contexts = list(self._active_recordings.values())

        for ctx in active_contexts:
            if not ctx.started or ctx.topic != topic:
                continue
            try:
                ref_pt, fb_pt, err_pt, label = pick_fields(msg)
                ctx.field_label = label
            except Exception as e:
                self.get_logger().error(str(e))
                continue

            n = len(getattr(ref_pt, "positions", []))
            if n == 0:
                continue
            ctx.ensure_buffers(n)

            t_rel = now - (ctx.start_time or now)
            ctx.t.append(t_rel)

            if len(ctx.t) >= 2:
                dt = ctx.t[-1] - ctx.t[-2]
                if 1e-6 < dt < 1.0:
                    ctx.dt_est = 0.98 * ctx.dt_est + 0.02 * dt if ctx.dt_est > 0 else dt

            fb_pos = list(getattr(fb_pt, "positions", []))
            fb_vel = list(getattr(fb_pt, "velocities", []))
            ref_pos = list(getattr(ref_pt, "positions", []))
            err_pos = list(getattr(err_pt, "positions", []))

            for j in range(n):
                ctx.ref[j].append(ref_pos[j] if j < len(ref_pos) else math.nan)
                ctx.fb[j].append(fb_pos[j] if j < len(fb_pos) else math.nan)
                ctx.err[j].append(err_pos[j] if j < len(err_pos) else math.nan)
                ctx.vel[j].append(fb_vel[j] if j < len(fb_vel) else math.nan)

            # FKマッピング
            if self._fk_ready and not ctx._joint_name_to_msg_index:
                msg_joint_names = list(getattr(msg, "joint_names", []))
                if msg_joint_names:
                    m = {}
                    missing = []
                    for jn in self._chain_joint_names:
                        if jn in msg_joint_names:
                            m[jn] = msg_joint_names.index(jn)
                        else:
                            missing.append(jn)
                    if not missing:
                        ctx._joint_name_to_msg_index = m

            # FK計算
            if self._fk_ready and self._fk_solver is not None and ctx._joint_name_to_msg_index:
                try:
                    joint_positions_ref = {}
                    joint_positions_fb  = {}
                    for jn in self._chain_joint_names:
                        idx = ctx._joint_name_to_msg_index[jn]
                        joint_positions_ref[jn] = ref_pos[idx] if idx < len(ref_pos) else 0.0
                        joint_positions_fb[jn]  = fb_pos[idx]  if idx < len(fb_pos)  else 0.0

                    ee_ref = self._fk_solver.compute(joint_positions_ref)
                    ee_fb  = self._fk_solver.compute(joint_positions_fb)

                    if ee_ref is None or ee_fb is None:
                        ctx.ee_ref[0].append(math.nan); ctx.ee_ref[1].append(math.nan); ctx.ee_ref[2].append(math.nan)
                        ctx.ee_fb[0].append(math.nan);  ctx.ee_fb[1].append(math.nan);  ctx.ee_fb[2].append(math.nan)
                        ctx.ee_err[0].append(math.nan); ctx.ee_err[1].append(math.nan); ctx.ee_err[2].append(math.nan)
                        ctx.ee_dist_err.append(math.nan)
                    else:
                        ex = ee_fb[0] - ee_ref[0]
                        ey = ee_fb[1] - ee_ref[1]
                        ez = ee_fb[2] - ee_ref[2]
                        dist = math.sqrt(ex*ex + ey*ey + ez*ez)
                        ctx.ee_ref[0].append(ee_ref[0]); ctx.ee_ref[1].append(ee_ref[1]); ctx.ee_ref[2].append(ee_ref[2])
                        ctx.ee_fb[0].append(ee_fb[0]);   ctx.ee_fb[1].append(ee_fb[1]);   ctx.ee_fb[2].append(ee_fb[2])
                        ctx.ee_err[0].append(ex);        ctx.ee_err[1].append(ey);        ctx.ee_err[2].append(ez)
                        ctx.ee_dist_err.append(dist)
                except Exception:
                    ctx.ee_ref[0].append(math.nan); ctx.ee_ref[1].append(math.nan); ctx.ee_ref[2].append(math.nan)
                    ctx.ee_fb[0].append(math.nan);  ctx.ee_fb[1].append(math.nan);  ctx.ee_fb[2].append(math.nan)
                    ctx.ee_err[0].append(math.nan); ctx.ee_err[1].append(math.nan); ctx.ee_err[2].append(math.nan)
                    ctx.ee_dist_err.append(math.nan)

    # ---------------------------
    # Plot & CSV
    # ---------------------------

    def save_csv(self, ctx: RecordingContext, path: str):
        """data.csvを保存（共通モジュールを使用）"""
        from .trajectory_analyzer import save_data_csv
        
        data = {
            't': ctx.t,
            'joints': {}
        }

        # ★EE（ref/fb/err）と距離誤差
        if len(ctx.ee_ref[0]) == len(ctx.t):
            data['ee'] = {
                'pos': {
                    'ref': ctx.ee_ref,
                    'fb':  ctx.ee_fb,
                    'err': ctx.ee_err,
                }
            }
        if len(ctx.ee_dist_err) == len(ctx.t):
            data['ee_dist_err'] = ctx.ee_dist_err
        
        for j in ctx.plot_joints:
            joint_name = self.urdf_joint_names[j] if j < len(self.urdf_joint_names) else f"j{j}"
            data['joints'][joint_name] = {
                'ref': ctx.ref[j],
                'fb': ctx.fb[j],
                'err': ctx.err[j],
                'vel': ctx.vel[j]
            }
        
        save_data_csv(data, path, self.urdf_joint_names)

    def save_plan_csv(self, ctx: RecordingContext, path: str):
        """Plan軌道専用のCSVを保存（共通モジュールを使用）"""
        from .trajectory_analyzer import save_plan_csv
        
        if not ctx.plan_t or not ctx.plan_pos:
            self.get_logger().warn("No plan data to save")
            return
        
        plan_data = {
            't': ctx.plan_t,
            'joints': {},
            'ee': ctx.ee_plan if len(ctx.ee_plan[0]) > 0 else None
        }
        
        for j in ctx.plot_joints:
            joint_name = self.urdf_joint_names[j] if j < len(self.urdf_joint_names) else f"j{j}"
            if ctx.plan_pos and j < len(ctx.plan_pos[0]):
                plan_data['joints'][joint_name] = [pos[j] for pos in ctx.plan_pos if j < len(pos)]
        
        save_plan_csv(plan_data, path, self.urdf_joint_names)

    def _finalize_and_save(self, ctx: RecordingContext) -> Tuple[bool, str]:
        if len(ctx.t) == 0:
            return False, "No samples recorded; nothing saved."

        if not ctx.save_files:
            return True, (
                f"save_files=False: processed {len(ctx.t)} samples in memory; "
                "no folder or file was created."
            )

        # ===== データを辞書形式に変換 =====
        data = {
            't': ctx.t,
            'joints': {}
        }

        # ★EE（ref/fb/err）と距離誤差
        if len(ctx.ee_ref[0]) == len(ctx.t):
            data['ee'] = {
                'pos': {
                    'ref': ctx.ee_ref,
                    'fb':  ctx.ee_fb,
                    'err': ctx.ee_err,
                }
            }
        if len(ctx.ee_dist_err) == len(ctx.t):
            data['ee_dist_err'] = ctx.ee_dist_err
        
        for j in ctx.plot_joints:
            joint_name = self.urdf_joint_names[j] if j < len(self.urdf_joint_names) else f"j{j}"
            data['joints'][joint_name] = {
                'ref': ctx.ref[j],
                'fb': ctx.fb[j],
                'err': ctx.err[j],
                'vel': ctx.vel[j]
            }
        
        # Planデータ
        plan_data = None
        if ctx.plan_t and ctx.plan_pos:
            plan_data = {
                't': ctx.plan_t,
                'joints': {},
                'ee': ctx.ee_plan if len(ctx.ee_plan[0]) > 0 else None
            }
            
            for j in ctx.plot_joints:
                joint_name = self.urdf_joint_names[j] if j < len(self.urdf_joint_names) else f"j{j}"
                if ctx.plan_pos and j < len(ctx.plan_pos[0]):
                    plan_data['joints'][joint_name] = [pos[j] for pos in ctx.plan_pos if j < len(pos)]
        
        # CSV保存
        self.save_csv(ctx, ctx.out_csv)
        self.save_plan_csv(ctx, ctx.out_plan_csv)
        
        # ★Link padding解析を実行
        urdf_path = str(self.get_parameter("urdf_path").value)
        if plan_data and plan_data['t'] and plan_data['joints']:
            try:
                from .trajectory_analyzer import analyze_link_padding
                
                self.get_logger().info("Starting link padding analysis...")
                analyze_link_padding(
                    data=data,
                    plan_data=plan_data,
                    analyzer=self.analyzer,
                    urdf_path=urdf_path,
                    output_dir=ctx.out_dir
                )
                self.get_logger().info(f"✓ Link padding analysis complete!")
            except Exception as e:
                self.get_logger().error(f"Failed to analyze link padding: {e}")
        
        return True, f"Saved results in {ctx.out_dir}"

    # ---------------------------
    # ros2 bag record -a
    # ---------------------------

    def _start_bag_record_all(self, ctx: RecordingContext):
        cmd = ["ros2", "bag", "record", "-a", "-o", ctx.bag_dir]
        self.get_logger().info(f"[{ctx.goal_id}] Starting bag: " + " ".join(cmd))

        bag_log_path = os.path.join(ctx.out_dir, "bag_record.log")
        ctx.bag_log = open(bag_log_path, "w", encoding="utf-8")

        ctx.bag_proc = subprocess.Popen(
            cmd,
            stdout=ctx.bag_log,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )

    def _stop_bag(self, ctx: RecordingContext):
        if ctx.bag_proc is None:
            return

        self.get_logger().info(f"[{ctx.goal_id}] Stopping bag (SIGINT)...")
        try:
            ctx.bag_proc.send_signal(signal.SIGINT)
            ctx.bag_proc.wait(timeout=15.0)
        except Exception:
            self.get_logger().warn(f"[{ctx.goal_id}] Bag did not stop gracefully; killing.")
            try:
                ctx.bag_proc.kill()
            except Exception:
                pass
        finally:
            ctx.bag_proc = None
            if ctx.bag_log is not None:
                try:
                    ctx.bag_log.close()
                except Exception:
                    pass
                ctx.bag_log = None


from rclpy.executors import MultiThreadedExecutor

def main():
    rclpy.init()
    node = TrajFollowRecordActionServer()

    executor = MultiThreadedExecutor(num_threads=2)  # 2以上推奨
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()