#!/usr/bin/env python3
"""
CSVデータを読み込んでJointStateを再生するシンプルなノード（2台のバックホウ）
ref (青): Plan目標値
fb (緑): Feedback実測値
"""
import os
import csv
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
from tf2_ros import TransformException


class VideoPlayerNode(Node):
    """CSVからJointStateを再生（2台：ref/fb）"""
    
    def __init__(self):
        super().__init__("video_player_node")
        
        # パラメータ
        self.declare_parameter("csv_path", "")
        self.declare_parameter("playback_speed", 1.0)
        self.declare_parameter("loop", False)
        self.declare_parameter("start_delay", 0.0)  # 再生開始の遅延時間
        self.declare_parameter("robot_namespace", "")  # namespace対応
        
        csv_path = str(self.get_parameter("csv_path").value)
        playback_speed = float(self.get_parameter("playback_speed").value)
        self.loop = bool(self.get_parameter("loop").value)
        start_delay = float(self.get_parameter("start_delay").value)
        robot_namespace = str(self.get_parameter("robot_namespace").value)
        
        # namespace用のプレフィックス（空の場合は空文字列、あればスラッシュ付き）
        self.ns_prefix = f"{robot_namespace}/" if robot_namespace else ""
        
        if not csv_path or not os.path.exists(csv_path):
            self.get_logger().error(f"CSV file not found: {csv_path}")
            raise FileNotFoundError(csv_path)
        
        # URDFの関節名（ベース名）
        # bucket_end_jointは実際には使われないため除外
        self.joint_names = [
            "swing_joint",
            "boom_joint", 
            "arm_joint",
            "bucket_joint",
        ]
        
        # プレフィックス付きの関節名（namespace対応）
        self.joint_names_ref = [f"{self.ns_prefix}ref/{name}" for name in self.joint_names]
        self.joint_names_fb = [f"{self.ns_prefix}fb/{name}" for name in self.joint_names]
        
        # Publishers（絶対パスで指定）
        self.pub_ref = self.create_publisher(JointState, "/video_gen/joint_states_ref", 10)
        self.pub_fb = self.create_publisher(JointState, "/video_gen/joint_states_fb", 10)
        
        # 軌跡パブリッシャー
        self.pub_path_ref = self.create_publisher(Path, "/video_gen/path_ref", 10)
        self.pub_path_fb = self.create_publisher(Path, "/video_gen/path_fb", 10)
        
        # PlanのEE位置マーカーパブリッシャー
        self.pub_plan_markers = self.create_publisher(MarkerArray, "/video_gen/plan_ee_markers", 10)
        
        # 対応点可視化用のマーカーパブリッシャー
        self.pub_comparison_markers = self.create_publisher(MarkerArray, "/video_gen/comparison_markers", 10)
        
        # 軌跡データ（刃先の位置履歴、namespace対応）
        self.path_ref = Path()
        self.path_ref.header.frame_id = f"{self.ns_prefix}ref/base_link"
        self.path_fb = Path()
        self.path_fb.header.frame_id = f"{self.ns_prefix}fb/base_link"
        
        # ★最近傍探索用のfeedback軌跡履歴（リスト形式で保持）
        self.fb_trajectory_history = []  # [(x, y, z), ...]
        
        # PlanのEE位置データ
        self.plan_ee_positions = []
        self._load_plan_ee_data()
        
        # Link correspondence データ
        self.link_correspondences = {}
        self._load_link_correspondence_data()
        
        # Link correspondence マーカーパブリッシャー
        self.pub_link_correspondence_markers = self.create_publisher(MarkerArray, "/video_gen/link_correspondence_markers", 10)
        
        # TFリスナー
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # データ読み込み
        self.data = self._load_csv(csv_path)
        self.frame_idx = 0
        
        # タイマー（30fps）
        timer_period = (1.0 / 30.0) / playback_speed
        
        # 遅延がある場合は、遅延後にタイマーを開始
        if start_delay > 0:
            self.get_logger().info(f"Waiting {start_delay} seconds before starting playback...")
            self.start_timer = self.create_timer(start_delay, self._start_playback)
            self.timer = None
        else:
            self.timer = self.create_timer(timer_period, self.publish_frame)
        
        self.timer_period = timer_period
        self.get_logger().info(f"Loaded {len(self.data)} frames from {csv_path}")
        self.get_logger().info(f"Playback speed: {playback_speed}x, Loop: {self.loop}")
    
    def _load_csv(self, csv_path: str):
        """CSVからデータ読み込み"""
        data = []
        with open(csv_path, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                data.append(row)
        return data
    
    def _load_plan_ee_data(self):
        """plan.csvからPlanのEE位置データを読み込み"""
        csv_path = str(self.get_parameter("csv_path").value)
        if not csv_path or not os.path.exists(csv_path):
            return
        
        # plan.csvのパスを推定（data.csvと同じディレクトリ）
        csv_dir = os.path.dirname(csv_path)
        plan_csv_path = os.path.join(csv_dir, "plan.csv")
        
        if not os.path.exists(plan_csv_path):
            self.get_logger().warn(f"plan.csv not found: {plan_csv_path}")
            return
        
        self.plan_ee_positions = []
        try:
            with open(plan_csv_path, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    # PlanのEE位置を読み込み
                    if 'ee_x' in row and 'ee_y' in row and 'ee_z' in row:
                        try:
                            x = float(row['ee_x'])
                            y = float(row['ee_y'])
                            z = float(row['ee_z'])
                            # nanでない場合のみ追加
                            if not (x != x or y != y or z != z):  # nanチェック
                                self.plan_ee_positions.append((x, y, z))
                        except (ValueError, KeyError):
                            continue
            
            if self.plan_ee_positions:
                self.get_logger().info(f"Loaded {len(self.plan_ee_positions)} plan EE positions from {plan_csv_path}")
                # PlanのEE位置マーカーを配信するタイマー（1秒ごと）
                self.create_timer(1.0, self._publish_plan_markers)
            else:
                self.get_logger().warn("No plan EE positions found in plan.csv")
        except Exception as e:
            self.get_logger().warn(f"Failed to load plan EE data: {e}")
    
    def _publish_plan_markers(self):
        """PlanのEE位置をマーカーとして配信（namespace対応）"""
        if not self.plan_ee_positions:
            return
        
        marker_array = MarkerArray()
        
        # 全てのPlanのEE位置を点として表示（namespace対応）
        marker = Marker()
        marker.header.frame_id = f"{self.ns_prefix}ref/base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "plan_ee"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05  # 点のサイズ
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.r = 1.0  # オレンジ色
        marker.color.g = 0.5
        marker.color.b = 0.0
        marker.color.a = 0.8
        
        for x, y, z in self.plan_ee_positions:
            from geometry_msgs.msg import Point
            p = Point()
            p.x = x
            p.y = y
            p.z = z
            marker.points.append(p)
        
        marker_array.markers.append(marker)
        
        # LINE_STRIPで軌跡を線として表示（namespace対応）
        line_marker = Marker()
        line_marker.header.frame_id = f"{self.ns_prefix}ref/base_link"
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.ns = "plan_ee_line"
        line_marker.id = 1
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.pose.orientation.w = 1.0
        line_marker.scale.x = 0.01  # 線の太さ
        line_marker.color.r = 1.0  # オレンジ色
        line_marker.color.g = 0.5
        line_marker.color.b = 0.0
        line_marker.color.a = 0.5
        
        for x, y, z in self.plan_ee_positions:
            from geometry_msgs.msg import Point
            p = Point()
            p.x = x
            p.y = y
            p.z = z
            line_marker.points.append(p)
        
        marker_array.markers.append(line_marker)
        
        self.pub_plan_markers.publish(marker_array)
    
    def _load_link_correspondence_data(self):
        """link_correspondence_nearest.csvからPlan-Feedback対応点を読み込み"""
        csv_path = str(self.get_parameter("csv_path").value)
        if not csv_path or not os.path.exists(csv_path):
            return
        
        # link_correspondence_nearest.csvのパスを推定
        csv_dir = os.path.dirname(csv_path)
        correspondence_csv_path = os.path.join(csv_dir, "link_correspondence_nearest.csv")
        
        if not os.path.exists(correspondence_csv_path):
            self.get_logger().warn(f"link_correspondence_nearest.csv not found: {correspondence_csv_path}")
            return
        
        # リンクごとにデータを格納
        # self.link_correspondences[link_name] = [(plan_pos, fb_pos, distance), ...]
        self.link_correspondences = {}
        
        try:
            with open(correspondence_csv_path, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    link_name = row['link_name']
                    
                    plan_pos = (
                        float(row['plan_x']),
                        float(row['plan_y']),
                        float(row['plan_z'])
                    )
                    
                    fb_pos = (
                        float(row['fb_nearest_x']),
                        float(row['fb_nearest_y']),
                        float(row['fb_nearest_z'])
                    )
                    
                    distance = float(row['distance_m'])
                    
                    if link_name not in self.link_correspondences:
                        self.link_correspondences[link_name] = []
                    
                    self.link_correspondences[link_name].append((plan_pos, fb_pos, distance))
            
            if self.link_correspondences:
                total_points = sum(len(corr) for corr in self.link_correspondences.values())
                self.get_logger().info(f"Loaded {total_points} correspondence points for {len(self.link_correspondences)} links")
                
                # Link correspondence マーカーを定期的に配信（1秒ごと）
                self.create_timer(1.0, self._publish_link_correspondence_markers)
            else:
                self.get_logger().warn("No correspondence data found")
                
        except Exception as e:
            self.get_logger().warn(f"Failed to load link correspondence data: {e}")
            import traceback
            traceback.print_exc()
    
    def _publish_link_correspondence_markers(self):
        """Link correspondenceをマーカーとして可視化（矢印で誤差ベクトル表示）"""
        if not self.link_correspondences:
            return
        
        marker_array = MarkerArray()
        marker_id = 0
        
        # 各リンクごとに可視化
        for link_name, correspondences in self.link_correspondences.items():
            if not correspondences:
                continue
            
            # リンクごとに色を変える
            colors = {
                'body_link': (0.0, 0.5, 1.0),    # 水色
                'boom_link': (1.0, 0.5, 0.0),    # オレンジ
                'arm_link': (1.0, 1.0, 0.0),     # 黄色
                'bucket_link': (1.0, 0.0, 1.0),  # マゼンタ
                'bucket_end_link': (1.0, 0.0, 0.5),  # ピンク
            }
            
            color = colors.get(link_name, (0.5, 0.5, 0.5))
            
            # Plan軌跡（点）
            plan_points_marker = Marker()
            plan_points_marker.header.frame_id = "world"
            plan_points_marker.header.stamp = self.get_clock().now().to_msg()
            plan_points_marker.ns = f"corr_plan_{link_name}"
            plan_points_marker.id = marker_id
            marker_id += 1
            plan_points_marker.type = Marker.SPHERE_LIST
            plan_points_marker.action = Marker.ADD
            plan_points_marker.pose.orientation.w = 1.0
            plan_points_marker.scale.x = 0.03
            plan_points_marker.scale.y = 0.03
            plan_points_marker.scale.z = 0.03
            plan_points_marker.color.r = color[0]
            plan_points_marker.color.g = color[1]
            plan_points_marker.color.b = color[2]
            plan_points_marker.color.a = 0.6
            
            from geometry_msgs.msg import Point
            for plan_pos, fb_pos, dist in correspondences:
                p = Point()
                p.x = plan_pos[0]
                p.y = plan_pos[1]
                p.z = plan_pos[2]
                plan_points_marker.points.append(p)
            
            marker_array.markers.append(plan_points_marker)
            
            # Feedback軌跡（点）
            fb_points_marker = Marker()
            fb_points_marker.header.frame_id = "world"
            fb_points_marker.header.stamp = self.get_clock().now().to_msg()
            fb_points_marker.ns = f"corr_fb_{link_name}"
            fb_points_marker.id = marker_id
            marker_id += 1
            fb_points_marker.type = Marker.SPHERE_LIST
            fb_points_marker.action = Marker.ADD
            fb_points_marker.pose.orientation.w = 1.0
            fb_points_marker.scale.x = 0.025
            fb_points_marker.scale.y = 0.025
            fb_points_marker.scale.z = 0.025
            fb_points_marker.color.r = 0.0
            fb_points_marker.color.g = 1.0
            fb_points_marker.color.b = 0.0
            fb_points_marker.color.a = 0.5
            
            for plan_pos, fb_pos, dist in correspondences:
                p = Point()
                p.x = fb_pos[0]
                p.y = fb_pos[1]
                p.z = fb_pos[2]
                fb_points_marker.points.append(p)
            
            marker_array.markers.append(fb_points_marker)
            
            # ★新規追加：矢印マーカーでPlan→Feedbackへの誤差ベクトルを表示（サンプリング）
            import math
            step = max(1, len(correspondences) // 30)  # 30本程度に間引き
            for i in range(0, len(correspondences), step):
                plan_pos, fb_pos, dist = correspondences[i]
                
                # 誤差が1mm以上ある場合のみ表示（ゼロベクトルは無視）
                if dist < 0.001:
                    continue
                
                arrow_marker = Marker()
                arrow_marker.header.frame_id = "world"
                arrow_marker.header.stamp = self.get_clock().now().to_msg()
                arrow_marker.ns = f"corr_arrows_{link_name}"
                arrow_marker.id = marker_id
                marker_id += 1
                arrow_marker.type = Marker.ARROW
                arrow_marker.action = Marker.ADD
                
                # 矢印の始点（Plan位置）と終点（Feedback位置）
                p_start = Point()
                p_start.x = plan_pos[0]
                p_start.y = plan_pos[1]
                p_start.z = plan_pos[2]
                
                p_end = Point()
                p_end.x = fb_pos[0]
                p_end.y = fb_pos[1]
                p_end.z = fb_pos[2]
                
                arrow_marker.points.append(p_start)
                arrow_marker.points.append(p_end)
                
                # 矢印のサイズ（誤差の大きさに応じてスケール）
                arrow_marker.scale.x = 0.01  # 軸の太さ
                arrow_marker.scale.y = 0.02  # 矢尻の太さ
                arrow_marker.scale.z = 0.03  # 矢尻の長さ
                
                # 色：誤差が大きいほど赤く（5cm以上で真っ赤）
                error_ratio = min(dist / 0.05, 1.0)
                arrow_marker.color.r = error_ratio
                arrow_marker.color.g = 1.0 - error_ratio
                arrow_marker.color.b = 0.0
                arrow_marker.color.a = 0.7
                
                marker_array.markers.append(arrow_marker)
            
            # 対応線（サンプリング）- 細い線で全体の対応を表示
            line_marker = Marker()
            line_marker.header.frame_id = "world"
            line_marker.header.stamp = self.get_clock().now().to_msg()
            line_marker.ns = f"corr_lines_{link_name}"
            line_marker.id = marker_id
            marker_id += 1
            line_marker.type = Marker.LINE_LIST
            line_marker.action = Marker.ADD
            line_marker.pose.orientation.w = 1.0
            line_marker.scale.x = 0.002  # より細い線
            line_marker.color.r = color[0]
            line_marker.color.g = color[1]
            line_marker.color.b = color[2]
            line_marker.color.a = 0.2  # より薄く
            
            # サンプリング（全部描くと多すぎるので、100点ごと）
            step = max(1, len(correspondences) // 100)
            for i in range(0, len(correspondences), step):
                plan_pos, fb_pos, dist = correspondences[i]
                
                p1 = Point()
                p1.x = plan_pos[0]
                p1.y = plan_pos[1]
                p1.z = plan_pos[2]
                line_marker.points.append(p1)
                
                p2 = Point()
                p2.x = fb_pos[0]
                p2.y = fb_pos[1]
                p2.z = fb_pos[2]
                line_marker.points.append(p2)
            
            marker_array.markers.append(line_marker)
            
            # 最大誤差点を強調表示
            max_corr = max(correspondences, key=lambda x: x[2])
            max_plan_pos, max_fb_pos, max_dist = max_corr
            
            # ★最大誤差の矢印（太く目立つ）
            max_arrow_marker = Marker()
            max_arrow_marker.header.frame_id = "world"
            max_arrow_marker.header.stamp = self.get_clock().now().to_msg()
            max_arrow_marker.ns = f"corr_max_arrow_{link_name}"
            max_arrow_marker.id = marker_id
            marker_id += 1
            max_arrow_marker.type = Marker.ARROW
            max_arrow_marker.action = Marker.ADD
            
            p_start = Point()
            p_start.x = max_plan_pos[0]
            p_start.y = max_plan_pos[1]
            p_start.z = max_plan_pos[2]
            
            p_end = Point()
            p_end.x = max_fb_pos[0]
            p_end.y = max_fb_pos[1]
            p_end.z = max_fb_pos[2]
            
            max_arrow_marker.points.append(p_start)
            max_arrow_marker.points.append(p_end)
            
            # 太い矢印
            max_arrow_marker.scale.x = 0.025  # 軸の太さ
            max_arrow_marker.scale.y = 0.04   # 矢尻の太さ
            max_arrow_marker.scale.z = 0.06   # 矢尻の長さ
            
            # 赤色で強調
            max_arrow_marker.color.r = 1.0
            max_arrow_marker.color.g = 0.0
            max_arrow_marker.color.b = 0.0
            max_arrow_marker.color.a = 1.0
            
            marker_array.markers.append(max_arrow_marker)
            
            # 最大誤差のテキスト表示
            text_marker = Marker()
            text_marker.header.frame_id = "world"
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = f"corr_text_{link_name}"
            text_marker.id = marker_id
            marker_id += 1
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = (max_plan_pos[0] + max_fb_pos[0]) / 2
            text_marker.pose.position.y = (max_plan_pos[1] + max_fb_pos[1]) / 2
            text_marker.pose.position.z = (max_plan_pos[2] + max_fb_pos[2]) / 2 + 0.2
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 0.1
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = f"{link_name}\nMax: {max_dist*1000:.1f}mm"
            
            marker_array.markers.append(text_marker)
            
            # ★リンクごとの統計情報を追加表示
            avg_dist = sum(d for _, _, d in correspondences) / len(correspondences)
            min_dist = min(d for _, _, d in correspondences)
            
            stats_text_marker = Marker()
            stats_text_marker.header.frame_id = "world"
            stats_text_marker.header.stamp = self.get_clock().now().to_msg()
            stats_text_marker.ns = f"corr_stats_{link_name}"
            stats_text_marker.id = marker_id
            marker_id += 1
            stats_text_marker.type = Marker.TEXT_VIEW_FACING
            stats_text_marker.action = Marker.ADD
            
            # 統計テキストの位置（リンクの中心付近）
            center_x = sum(p[0] for p, _, _ in correspondences) / len(correspondences)
            center_y = sum(p[1] for p, _, _ in correspondences) / len(correspondences)
            center_z = sum(p[2] for p, _, _ in correspondences) / len(correspondences)
            
            stats_text_marker.pose.position.x = center_x
            stats_text_marker.pose.position.y = center_y
            stats_text_marker.pose.position.z = center_z + 0.5
            stats_text_marker.pose.orientation.w = 1.0
            stats_text_marker.scale.z = 0.08
            stats_text_marker.color.r = color[0]
            stats_text_marker.color.g = color[1]
            stats_text_marker.color.b = color[2]
            stats_text_marker.color.a = 0.9
            stats_text_marker.text = (
                f"{link_name}\n"
                f"Avg: {avg_dist*1000:.1f}mm\n"
                f"Min: {min_dist*1000:.1f}mm\n"
                f"Max: {max_dist*1000:.1f}mm"
            )
            
            marker_array.markers.append(stats_text_marker)
        
        self.pub_link_correspondence_markers.publish(marker_array)
    
    def publish_frame(self):
        """1フレーム分のJointStateを配信（2台：ref/fb）"""
        if self.frame_idx >= len(self.data):
            if self.loop:
                # ループ時にPathと履歴をリセット
                self.path_ref.poses.clear()
                self.path_fb.poses.clear()
                self.fb_trajectory_history.clear()  # ★履歴もリセット
                self.get_logger().info("Looping playback... (Path and history reset)")
                self.frame_idx = 0
            else:
                self.get_logger().info("Playback finished")
                self.timer.cancel()
                # 3秒待ってからシャットダウン
                import time
                time.sleep(3)
                self.get_logger().info("Shutting down...")
                import sys
                sys.exit(0)
                return
        
        row = self.data[self.frame_idx]
        
        # JointState作成（ref）
        msg_ref = JointState()
        msg_ref.header.stamp = self.get_clock().now().to_msg()
        msg_ref.header.frame_id = ""
        msg_ref.name = self.joint_names_ref
        msg_ref.position = []
        
        # JointState作成（fb）
        msg_fb = JointState()
        msg_fb.header.stamp = self.get_clock().now().to_msg()
        msg_fb.header.frame_id = ""
        msg_fb.name = self.joint_names_fb
        msg_fb.position = []
        
        # CSVから値を読み取る
        for joint_name in self.joint_names:
            ref_key = f"{joint_name}_ref"
            fb_key = f"{joint_name}_fb"
            
            if ref_key in row:
                msg_ref.position.append(float(row[ref_key]))
            
            if fb_key in row:
                msg_fb.position.append(float(row[fb_key]))
        
        # 配信
        self.pub_ref.publish(msg_ref)
        self.pub_fb.publish(msg_fb)
        
        # 刃先の位置をCSVから直接取得して軌跡に追加
        self._update_trajectory_from_csv(row)
        
        self.frame_idx += 1
    
    def _update_trajectory_from_csv(self, row):
        """CSVから直接刃先の位置を取得して軌跡を更新（2台：ref/fb）"""
        try:
            # CSVから刃先の位置を読み取る
            ref_pos = None
            fb_pos = None
            
            # Reference
            if 'ee_ref_x' in row and 'ee_ref_y' in row and 'ee_ref_z' in row:
                pose_ref = PoseStamped()
                pose_ref.header.stamp = self.get_clock().now().to_msg()
                pose_ref.header.frame_id = f"{self.ns_prefix}ref/base_link"
                pose_ref.pose.position.x = float(row['ee_ref_x'])
                pose_ref.pose.position.y = float(row['ee_ref_y'])
                pose_ref.pose.position.z = float(row['ee_ref_z'])
                pose_ref.pose.orientation.w = 1.0
                
                ref_pos = (pose_ref.pose.position.x, pose_ref.pose.position.y, pose_ref.pose.position.z)
                
                self.path_ref.poses.append(pose_ref)
                self.path_ref.header.stamp = self.get_clock().now().to_msg()
                self.pub_path_ref.publish(self.path_ref)
            
            # Feedback
            if 'ee_fb_x' in row and 'ee_fb_y' in row and 'ee_fb_z' in row:
                pose_fb = PoseStamped()
                pose_fb.header.stamp = self.get_clock().now().to_msg()
                pose_fb.header.frame_id = f"{self.ns_prefix}fb/base_link"
                pose_fb.pose.position.x = float(row['ee_fb_x'])
                pose_fb.pose.position.y = float(row['ee_fb_y'])
                pose_fb.pose.position.z = float(row['ee_fb_z'])
                pose_fb.pose.orientation.w = 1.0
                
                fb_pos = (pose_fb.pose.position.x, pose_fb.pose.position.y, pose_fb.pose.position.z)
                
                # ★Feedback軌跡履歴に追加
                self.fb_trajectory_history.append(fb_pos)
                
                self.path_fb.poses.append(pose_fb)
                self.path_fb.header.stamp = self.get_clock().now().to_msg()
                self.pub_path_fb.publish(self.path_fb)
            
            # ★最近傍対応の可視化（ref vs fb最近傍）
            if ref_pos is not None and len(self.fb_trajectory_history) > 0:
                self._publish_comparison_markers_nearest(ref_pos)
                
        except (KeyError, ValueError) as e:
            # CSVにEE位置データがない場合は無視
            pass
    
    def _publish_comparison_markers(self, ref_pos, fb_pos):
        """ref vs fbの対応点を可視化（2台）"""
        marker_array = MarkerArray()
        
        # ref-fb間の線（黄色）
        line_marker_fb = Marker()
        line_marker_fb.header.frame_id = "world"
        line_marker_fb.header.stamp = self.get_clock().now().to_msg()
        line_marker_fb.ns = "comparison_line_fb"
        line_marker_fb.id = 0
        line_marker_fb.type = Marker.LINE_LIST
        line_marker_fb.action = Marker.ADD
        line_marker_fb.pose.orientation.w = 1.0
        line_marker_fb.scale.x = 0.015  # 線の太さ
        line_marker_fb.color.r = 1.0
        line_marker_fb.color.g = 1.0
        line_marker_fb.color.b = 0.0
        line_marker_fb.color.a = 0.6
        
        from geometry_msgs.msg import Point
        p1 = Point()
        p1.x = ref_pos[0]
        p1.y = ref_pos[1]
        p1.z = ref_pos[2]
        line_marker_fb.points.append(p1)
        
        p2 = Point()
        p2.x = fb_pos[0]
        p2.y = fb_pos[1]
        p2.z = fb_pos[2]
        line_marker_fb.points.append(p2)
        
        marker_array.markers.append(line_marker_fb)
        
        # ref点のマーカー（青い球）
        ref_sphere = Marker()
        ref_sphere.header.frame_id = "world"
        ref_sphere.header.stamp = self.get_clock().now().to_msg()
        ref_sphere.ns = "comparison_ref"
        ref_sphere.id = 1
        ref_sphere.type = Marker.SPHERE
        ref_sphere.action = Marker.ADD
        ref_sphere.pose.position.x = ref_pos[0]
        ref_sphere.pose.position.y = ref_pos[1]
        ref_sphere.pose.position.z = ref_pos[2]
        ref_sphere.pose.orientation.w = 1.0
        ref_sphere.scale.x = 0.08
        ref_sphere.scale.y = 0.08
        ref_sphere.scale.z = 0.08
        ref_sphere.color.r = 0.0
        ref_sphere.color.g = 0.0
        ref_sphere.color.b = 1.0
        ref_sphere.color.a = 0.9
        
        marker_array.markers.append(ref_sphere)
        
        # fb点のマーカー（緑の球）
        fb_sphere = Marker()
        fb_sphere.header.frame_id = "world"
        fb_sphere.header.stamp = self.get_clock().now().to_msg()
        fb_sphere.ns = "comparison_fb"
        fb_sphere.id = 2
        fb_sphere.type = Marker.SPHERE
        fb_sphere.action = Marker.ADD
        fb_sphere.pose.position.x = fb_pos[0]
        fb_sphere.pose.position.y = fb_pos[1]
        fb_sphere.pose.position.z = fb_pos[2]
        fb_sphere.pose.orientation.w = 1.0
        fb_sphere.scale.x = 0.08
        fb_sphere.scale.y = 0.08
        fb_sphere.scale.z = 0.08
        fb_sphere.color.r = 0.0
        fb_sphere.color.g = 1.0
        fb_sphere.color.b = 0.0
        fb_sphere.color.a = 0.9
        
        marker_array.markers.append(fb_sphere)
        
        # 誤差テキスト表示（refの上に表示）
        import math
        dist_fb = math.sqrt((ref_pos[0] - fb_pos[0])**2 + 
                           (ref_pos[1] - fb_pos[1])**2 + 
                           (ref_pos[2] - fb_pos[2])**2)
        
        text_marker = Marker()
        text_marker.header.frame_id = "world"
        text_marker.header.stamp = self.get_clock().now().to_msg()
        text_marker.ns = "comparison_text"
        text_marker.id = 3
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        # refの位置から上方に配置（見やすくする）
        text_marker.pose.position.x = ref_pos[0]
        text_marker.pose.position.y = ref_pos[1]
        text_marker.pose.position.z = ref_pos[2] + 0.5  # refの50cm上
        text_marker.pose.orientation.w = 1.0
        text_marker.scale.z = 0.15
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        text_marker.text = f"Error: {dist_fb*1000:.1f}mm"
        
        marker_array.markers.append(text_marker)
        
        self.pub_comparison_markers.publish(marker_array)
    
    def _publish_comparison_markers_nearest(self, ref_pos):
        """ref vs fbの最近傍対応点を可視化（最近傍探索版）"""
        if not self.fb_trajectory_history:
            return
        
        # ★現在のref位置に対してfeedback軌跡全体から最近傍点を探す
        import math
        min_dist = float('inf')
        nearest_fb_pos = None
        
        for fb_pos in self.fb_trajectory_history:
            dist = math.sqrt(
                (ref_pos[0] - fb_pos[0])**2 + 
                (ref_pos[1] - fb_pos[1])**2 + 
                (ref_pos[2] - fb_pos[2])**2
            )
            if dist < min_dist:
                min_dist = dist
                nearest_fb_pos = fb_pos
        
        if nearest_fb_pos is None:
            return
        
        marker_array = MarkerArray()
        
        # ref-fb最近傍間の矢印（誤差ベクトル）
        from geometry_msgs.msg import Point
        arrow_marker = Marker()
        arrow_marker.header.frame_id = "world"
        arrow_marker.header.stamp = self.get_clock().now().to_msg()
        arrow_marker.ns = "comparison_arrow_nearest"
        arrow_marker.id = 0
        arrow_marker.type = Marker.ARROW
        arrow_marker.action = Marker.ADD
        
        p_start = Point()
        p_start.x = ref_pos[0]
        p_start.y = ref_pos[1]
        p_start.z = ref_pos[2]
        
        p_end = Point()
        p_end.x = nearest_fb_pos[0]
        p_end.y = nearest_fb_pos[1]
        p_end.z = nearest_fb_pos[2]
        
        arrow_marker.points.append(p_start)
        arrow_marker.points.append(p_end)
        
        # 矢印のサイズ
        arrow_marker.scale.x = 0.02  # 軸の太さ
        arrow_marker.scale.y = 0.04  # 矢尻の太さ
        arrow_marker.scale.z = 0.06  # 矢尻の長さ
        
        # 色：誤差が大きいほど赤く（5cm以上で真っ赤）
        error_ratio = min(min_dist / 0.05, 1.0)
        arrow_marker.color.r = error_ratio
        arrow_marker.color.g = 1.0 - error_ratio
        arrow_marker.color.b = 0.0
        arrow_marker.color.a = 0.8
        
        marker_array.markers.append(arrow_marker)
        
        # ref点のマーカー（青い球）
        ref_sphere = Marker()
        ref_sphere.header.frame_id = "world"
        ref_sphere.header.stamp = self.get_clock().now().to_msg()
        ref_sphere.ns = "comparison_ref_nearest"
        ref_sphere.id = 1
        ref_sphere.type = Marker.SPHERE
        ref_sphere.action = Marker.ADD
        ref_sphere.pose.position.x = ref_pos[0]
        ref_sphere.pose.position.y = ref_pos[1]
        ref_sphere.pose.position.z = ref_pos[2]
        ref_sphere.pose.orientation.w = 1.0
        ref_sphere.scale.x = 0.1
        ref_sphere.scale.y = 0.1
        ref_sphere.scale.z = 0.1
        ref_sphere.color.r = 0.0
        ref_sphere.color.g = 0.0
        ref_sphere.color.b = 1.0
        ref_sphere.color.a = 0.9
        
        marker_array.markers.append(ref_sphere)
        
        # fb最近傍点のマーカー（緑の球）
        fb_sphere = Marker()
        fb_sphere.header.frame_id = "world"
        fb_sphere.header.stamp = self.get_clock().now().to_msg()
        fb_sphere.ns = "comparison_fb_nearest"
        fb_sphere.id = 2
        fb_sphere.type = Marker.SPHERE
        fb_sphere.action = Marker.ADD
        fb_sphere.pose.position.x = nearest_fb_pos[0]
        fb_sphere.pose.position.y = nearest_fb_pos[1]
        fb_sphere.pose.position.z = nearest_fb_pos[2]
        fb_sphere.pose.orientation.w = 1.0
        fb_sphere.scale.x = 0.1
        fb_sphere.scale.y = 0.1
        fb_sphere.scale.z = 0.1
        fb_sphere.color.r = 0.0
        fb_sphere.color.g = 1.0
        fb_sphere.color.b = 0.0
        fb_sphere.color.a = 0.9
        
        marker_array.markers.append(fb_sphere)
        
        # 誤差テキスト表示（refの上に表示）
        text_marker = Marker()
        text_marker.header.frame_id = "world"
        text_marker.header.stamp = self.get_clock().now().to_msg()
        text_marker.ns = "comparison_text_nearest"
        text_marker.id = 3
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        # refの位置から上方に配置（見やすくする）
        text_marker.pose.position.x = ref_pos[0]
        text_marker.pose.position.y = ref_pos[1]
        text_marker.pose.position.z = ref_pos[2] + 0.5  # refの50cm上
        text_marker.pose.orientation.w = 1.0
        text_marker.scale.z = 0.15
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        text_marker.text = f"Nearest Error: {min_dist*1000:.1f}mm"
        
        marker_array.markers.append(text_marker)
        
        self.pub_comparison_markers.publish(marker_array)
    
    def _start_playback(self):
        """遅延後に再生を開始"""
        self.get_logger().info("Starting playback now!")
        self.start_timer.cancel()
        self.timer = self.create_timer(self.timer_period, self.publish_frame)


def main(args=None):
    rclpy.init(args=args)
    node = VideoPlayerNode()
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
