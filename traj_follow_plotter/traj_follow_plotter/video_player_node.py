#!/usr/bin/env python3
"""
CSVデータを読み込んでJointStateを再生するシンプルなノード（3台のバックホウ対応）
ref (青): 目標値
fb (緑): 元のfeedback
fb_comp (赤): 補正済みfeedback
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
    """CSVからJointStateを再生（3台対応）"""
    
    def __init__(self):
        super().__init__("video_player_node")
        
        # パラメータ
        self.declare_parameter("csv_path", "")
        self.declare_parameter("playback_speed", 1.0)
        self.declare_parameter("loop", False)
        self.declare_parameter("start_delay", 0.0)  # 再生開始の遅延時間
        self.declare_parameter("use_compensated", False)  # 補正済みデータを使用するか
        self.declare_parameter("robot_namespace", "")  # ★新規追加：namespace対応
        
        csv_path = str(self.get_parameter("csv_path").value)
        playback_speed = float(self.get_parameter("playback_speed").value)
        self.loop = bool(self.get_parameter("loop").value)
        start_delay = float(self.get_parameter("start_delay").value)
        use_compensated = bool(self.get_parameter("use_compensated").value)
        robot_namespace = str(self.get_parameter("robot_namespace").value)
        
        # namespace用のプレフィックス（空の場合は空文字列、あればスラッシュ付き）
        self.ns_prefix = f"{robot_namespace}/" if robot_namespace else ""
        
        # 補正済みCSVが存在する場合はそちらを使用
        if use_compensated:
            csv_dir = os.path.dirname(csv_path)
            compensated_csv = os.path.join(csv_dir, 'data_compensated.csv')
            if os.path.exists(compensated_csv):
                csv_path = compensated_csv
                self.get_logger().info(f"Using compensated CSV: {compensated_csv}")
            else:
                self.get_logger().warn(f"Compensated CSV not found: {compensated_csv}, using original data")
        
        if not csv_path or not os.path.exists(csv_path):
            self.get_logger().error(f"CSV file not found: {csv_path}")
            raise FileNotFoundError(csv_path)
        
        # URDFの関節名（ベース名）
        self.joint_names = [
            "swing_joint",
            "boom_joint", 
            "arm_joint",
            "bucket_joint",
            "bucket_end_joint"
        ]
        
        # プレフィックス付きの関節名（namespace対応）
        # 例: robot_namespace="robot1" の場合 → "robot1/ref/swing_joint"
        self.joint_names_ref = [f"{self.ns_prefix}ref/{name}" for name in self.joint_names]
        self.joint_names_fb = [f"{self.ns_prefix}fb/{name}" for name in self.joint_names]
        self.joint_names_fb_comp = [f"{self.ns_prefix}fb_comp/{name}" for name in self.joint_names]
        
        # Publishers（絶対パスで指定）
        self.pub_ref = self.create_publisher(JointState, "/video_gen/joint_states_ref", 10)
        self.pub_fb = self.create_publisher(JointState, "/video_gen/joint_states_fb", 10)
        self.pub_fb_comp = self.create_publisher(JointState, "/video_gen/joint_states_fb_comp", 10)  # ★新規追加
        
        # 軌跡パブリッシャー
        self.pub_path_ref = self.create_publisher(Path, "/video_gen/path_ref", 10)
        self.pub_path_fb = self.create_publisher(Path, "/video_gen/path_fb", 10)
        self.pub_path_fb_comp = self.create_publisher(Path, "/video_gen/path_fb_comp", 10)  # ★新規追加
        
        # PlanのEE位置マーカーパブリッシャー
        self.pub_plan_markers = self.create_publisher(MarkerArray, "/video_gen/plan_ee_markers", 10)
        
        # 対応点可視化用のマーカーパブリッシャー
        self.pub_comparison_markers = self.create_publisher(MarkerArray, "/video_gen/comparison_markers", 10)
        
        # 軌跡データ（刃先の位置履歴、namespace対応）
        self.path_ref = Path()
        self.path_ref.header.frame_id = f"{self.ns_prefix}ref/base_link"
        self.path_fb = Path()
        self.path_fb.header.frame_id = f"{self.ns_prefix}fb/base_link"
        self.path_fb_comp = Path()
        self.path_fb_comp.header.frame_id = f"{self.ns_prefix}fb_comp/base_link"
        
        # PlanのEE位置データ
        self.plan_ee_positions = []
        self._load_plan_ee_data()
        
        # TFリスナー
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # データ読み込み
        self.data = self._load_csv(csv_path)
        self.frame_idx = 0
        
        # 補正済みデータが利用可能かチェック
        self.has_compensated_data = self._check_compensated_data()
        
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
        marker.header.frame_id = f"{self.ns_prefix}ref/base_link"  # ★namespace対応
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
        line_marker.header.frame_id = f"{self.ns_prefix}ref/base_link"  # ★namespace対応
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
    
    def _check_compensated_data(self):
        """補正済みデータが利用可能かチェック"""
        if not self.data:
            return False
        
        # 最初の行に補正済み関節角度があるかチェック
        row = self.data[0]
        for joint_name in self.joint_names:
            comp_key = f"{joint_name}_fb_compensated"
            if comp_key not in row:
                self.get_logger().info("Compensated joint data not found in CSV")
                return False
        
        self.get_logger().info("✓ Compensated joint data available")
        
        # 補正済みEE位置があるかチェック
        if 'ee_fb_compensated_x' in row and 'ee_fb_compensated_y' in row and 'ee_fb_compensated_z' in row:
            self.get_logger().info("✓ Compensated EE position data available")
        else:
            self.get_logger().info("Compensated EE position data not found (will be computed from joints)")
        
        return True
    
    def publish_frame(self):
        """1フレーム分のJointStateを配信（3台対応）"""
        if self.frame_idx >= len(self.data):
            if self.loop:
                # ループ時にPathをリセット
                self.path_ref.poses.clear()
                self.path_fb.poses.clear()
                self.path_fb_comp.poses.clear()  # ★新規追加
                self.get_logger().info("Looping playback... (Path reset)")
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
        
        # JointState作成（fb_comp）★新規追加
        msg_fb_comp = JointState()
        msg_fb_comp.header.stamp = self.get_clock().now().to_msg()
        msg_fb_comp.header.frame_id = ""
        msg_fb_comp.name = self.joint_names_fb_comp
        msg_fb_comp.position = []
        
        # CSVから値を読み取る
        for joint_name in self.joint_names:
            ref_key = f"{joint_name}_ref"
            fb_key = f"{joint_name}_fb"
            fb_comp_key = f"{joint_name}_fb_compensated"
            
            if ref_key in row:
                msg_ref.position.append(float(row[ref_key]))
            
            if fb_key in row:
                msg_fb.position.append(float(row[fb_key]))
            
            # 補正済みデータがあれば使用、なければfbと同じ値を使用
            if self.has_compensated_data and fb_comp_key in row:
                msg_fb_comp.position.append(float(row[fb_comp_key]))
            elif fb_key in row:
                msg_fb_comp.position.append(float(row[fb_key]))
        
        # 配信
        self.pub_ref.publish(msg_ref)
        self.pub_fb.publish(msg_fb)
        self.pub_fb_comp.publish(msg_fb_comp)  # ★新規追加
        
        # 刃先の位置をCSVから直接取得して軌跡に追加
        self._update_trajectory_from_csv(row)
        
        self.frame_idx += 1
    
    def _update_trajectory_from_csv(self, row):
        """CSVから直接刃先の位置を取得して軌跡を更新（3台対応）"""
        try:
            # CSVから刃先の位置を読み取る
            ref_pos = None
            fb_pos = None
            fb_comp_pos = None
            
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
                
                self.path_fb.poses.append(pose_fb)
                self.path_fb.header.stamp = self.get_clock().now().to_msg()
                self.pub_path_fb.publish(self.path_fb)
            
            # ★新規追加：Feedback Compensated
            if self.has_compensated_data and 'ee_fb_compensated_x' in row and 'ee_fb_compensated_y' in row and 'ee_fb_compensated_z' in row:
                pose_fb_comp = PoseStamped()
                pose_fb_comp.header.stamp = self.get_clock().now().to_msg()
                pose_fb_comp.header.frame_id = f"{self.ns_prefix}fb_comp/base_link"
                pose_fb_comp.pose.position.x = float(row['ee_fb_compensated_x'])
                pose_fb_comp.pose.position.y = float(row['ee_fb_compensated_y'])
                pose_fb_comp.pose.position.z = float(row['ee_fb_compensated_z'])
                pose_fb_comp.pose.orientation.w = 1.0
                
                fb_comp_pos = (pose_fb_comp.pose.position.x, pose_fb_comp.pose.position.y, pose_fb_comp.pose.position.z)
                
                self.path_fb_comp.poses.append(pose_fb_comp)
                self.path_fb_comp.header.stamp = self.get_clock().now().to_msg()
                self.pub_path_fb_comp.publish(self.path_fb_comp)
            
            # 対応点の可視化（ref、fb、fb_compを比較）
            if ref_pos is not None and fb_pos is not None:
                self._publish_comparison_markers(ref_pos, fb_pos, fb_comp_pos)
                
        except (KeyError, ValueError) as e:
            # CSVにEE位置データがない場合は無視
            pass
    
    def _publish_comparison_markers(self, ref_pos, fb_pos, fb_comp_pos=None):
        """ref、fb、fb_compの対応点を可視化（3台対応）"""
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
        
        # ★新規追加：ref-fb_comp間の線（マゼンタ）
        if fb_comp_pos is not None:
            line_marker_comp = Marker()
            line_marker_comp.header.frame_id = "world"
            line_marker_comp.header.stamp = self.get_clock().now().to_msg()
            line_marker_comp.ns = "comparison_line_comp"
            line_marker_comp.id = 1
            line_marker_comp.type = Marker.LINE_LIST
            line_marker_comp.action = Marker.ADD
            line_marker_comp.pose.orientation.w = 1.0
            line_marker_comp.scale.x = 0.015
            line_marker_comp.color.r = 1.0
            line_marker_comp.color.g = 0.0
            line_marker_comp.color.b = 1.0
            line_marker_comp.color.a = 0.8
            
            p1_comp = Point()
            p1_comp.x = ref_pos[0]
            p1_comp.y = ref_pos[1]
            p1_comp.z = ref_pos[2]
            line_marker_comp.points.append(p1_comp)
            
            p2_comp = Point()
            p2_comp.x = fb_comp_pos[0]
            p2_comp.y = fb_comp_pos[1]
            p2_comp.z = fb_comp_pos[2]
            line_marker_comp.points.append(p2_comp)
            
            marker_array.markers.append(line_marker_comp)
        
        # ref点のマーカー（青い球）
        ref_sphere = Marker()
        ref_sphere.header.frame_id = "world"
        ref_sphere.header.stamp = self.get_clock().now().to_msg()
        ref_sphere.ns = "comparison_ref"
        ref_sphere.id = 2
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
        fb_sphere.id = 3
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
        
        # ★新規追加：fb_comp点のマーカー（赤い球）
        if fb_comp_pos is not None:
            fb_comp_sphere = Marker()
            fb_comp_sphere.header.frame_id = "world"
            fb_comp_sphere.header.stamp = self.get_clock().now().to_msg()
            fb_comp_sphere.ns = "comparison_fb_comp"
            fb_comp_sphere.id = 4
            fb_comp_sphere.type = Marker.SPHERE
            fb_comp_sphere.action = Marker.ADD
            fb_comp_sphere.pose.position.x = fb_comp_pos[0]
            fb_comp_sphere.pose.position.y = fb_comp_pos[1]
            fb_comp_sphere.pose.position.z = fb_comp_pos[2]
            fb_comp_sphere.pose.orientation.w = 1.0
            fb_comp_sphere.scale.x = 0.08
            fb_comp_sphere.scale.y = 0.08
            fb_comp_sphere.scale.z = 0.08
            fb_comp_sphere.color.r = 1.0
            fb_comp_sphere.color.g = 0.0
            fb_comp_sphere.color.b = 0.0
            fb_comp_sphere.color.a = 0.9
            
            marker_array.markers.append(fb_comp_sphere)
        
        # 誤差テキスト表示（refの上に表示）
        import math
        dist_fb = math.sqrt((ref_pos[0] - fb_pos[0])**2 + 
                           (ref_pos[1] - fb_pos[1])**2 + 
                           (ref_pos[2] - fb_pos[2])**2)
        
        text_marker = Marker()
        text_marker.header.frame_id = "world"
        text_marker.header.stamp = self.get_clock().now().to_msg()
        text_marker.ns = "comparison_text"
        text_marker.id = 5
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
        
        if fb_comp_pos is not None:
            dist_comp = math.sqrt((ref_pos[0] - fb_comp_pos[0])**2 + 
                                 (ref_pos[1] - fb_comp_pos[1])**2 + 
                                 (ref_pos[2] - fb_comp_pos[2])**2)
            text_marker.text = f"FB: {dist_fb:.4f}m\nComp: {dist_comp:.4f}m"
        else:
            text_marker.text = f"Error: {dist_fb:.4f}m"
        
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
