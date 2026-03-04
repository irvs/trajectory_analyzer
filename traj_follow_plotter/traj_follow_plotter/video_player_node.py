#!/usr/bin/env python3
"""
CSVデータを読み込んでJointStateを再生するシンプルなノード
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
    """CSVからJointStateを再生"""
    
    def __init__(self):
        super().__init__("video_player_node")
        
        # パラメータ
        self.declare_parameter("csv_path", "")
        self.declare_parameter("playback_speed", 1.0)
        self.declare_parameter("loop", False)
        self.declare_parameter("start_delay", 0.0)  # 再生開始の遅延時間
        self.declare_parameter("use_compensated", False)  # 補正済みデータを使用するか
        
        csv_path = str(self.get_parameter("csv_path").value)
        playback_speed = float(self.get_parameter("playback_speed").value)
        self.loop = bool(self.get_parameter("loop").value)
        start_delay = float(self.get_parameter("start_delay").value)
        use_compensated = bool(self.get_parameter("use_compensated").value)
        
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
        
        # プレフィックス付きの関節名
        self.joint_names_ref = [f"ref/{name}" for name in self.joint_names]
        self.joint_names_fb = [f"fb/{name}" for name in self.joint_names]
        
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
        
        # 軌跡データ（刃先の位置履歴）
        self.path_ref = Path()
        self.path_ref.header.frame_id = "ref/base_link"
        self.path_fb = Path()
        self.path_fb.header.frame_id = "fb/base_link"
        
        # PlanのEE位置データ
        self.plan_ee_positions = []
        self._load_plan_ee_data()
        
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
        """PlanのEE位置をマーカーとして配信"""
        if not self.plan_ee_positions:
            return
        
        marker_array = MarkerArray()
        
        # 全てのPlanのEE位置を点として表示
        marker = Marker()
        marker.header.frame_id = "ref/base_link"
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
        
        # LINE_STRIPで軌跡を線として表示
        line_marker = Marker()
        line_marker.header.frame_id = "ref/base_link"
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
    
    def publish_frame(self):
        """1フレーム分のJointStateを配信"""
        if self.frame_idx >= len(self.data):
            if self.loop:
                # ループ時にPathをリセット（フレームインデックスをリセットする前に）
                self.path_ref.poses.clear()
                self.path_fb.poses.clear()
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
        
        # JointState作成
        msg_ref = JointState()
        msg_ref.header.stamp = self.get_clock().now().to_msg()
        msg_ref.header.frame_id = ""
        msg_ref.name = self.joint_names_ref
        msg_ref.position = []
        
        msg_fb = JointState()
        msg_fb.header.stamp = self.get_clock().now().to_msg()
        msg_fb.header.frame_id = ""
        msg_fb.name = self.joint_names_fb
        msg_fb.position = []
        
        # CSVから値を読み取る
        for joint_name in self.joint_names:
            ref_key = f"{joint_name}_ref"
            fb_key = f"{joint_name}_fb"
            
            if ref_key in row and fb_key in row:
                msg_ref.position.append(float(row[ref_key]))
                msg_fb.position.append(float(row[fb_key]))
        
        # 配信
        self.pub_ref.publish(msg_ref)
        self.pub_fb.publish(msg_fb)
        
        # 刃先の位置をCSVから直接取得して軌跡に追加（TFを使わない）
        self._update_trajectory_from_csv(row)
        
        self.frame_idx += 1
    
    def _update_trajectory_from_csv(self, row):
        """CSVから直接刃先の位置を取得して軌跡を更新（高速再生でも安定）"""
        try:
            # CSVから刃先の位置を読み取る
            ref_pos = None
            fb_pos = None
            
            if 'ee_ref_x' in row and 'ee_ref_y' in row and 'ee_ref_z' in row:
                pose_ref = PoseStamped()
                pose_ref.header.stamp = self.get_clock().now().to_msg()
                pose_ref.header.frame_id = "ref/base_link"
                pose_ref.pose.position.x = float(row['ee_ref_x'])
                pose_ref.pose.position.y = float(row['ee_ref_y'])
                pose_ref.pose.position.z = float(row['ee_ref_z'])
                pose_ref.pose.orientation.w = 1.0
                
                ref_pos = (pose_ref.pose.position.x, pose_ref.pose.position.y, pose_ref.pose.position.z)
                
                self.path_ref.poses.append(pose_ref)
                self.path_ref.header.stamp = self.get_clock().now().to_msg()
                self.pub_path_ref.publish(self.path_ref)
            
            if 'ee_fb_x' in row and 'ee_fb_y' in row and 'ee_fb_z' in row:
                pose_fb = PoseStamped()
                pose_fb.header.stamp = self.get_clock().now().to_msg()
                pose_fb.header.frame_id = "fb/base_link"
                pose_fb.pose.position.x = float(row['ee_fb_x'])
                pose_fb.pose.position.y = float(row['ee_fb_y'])
                pose_fb.pose.position.z = float(row['ee_fb_z'])
                pose_fb.pose.orientation.w = 1.0
                
                fb_pos = (pose_fb.pose.position.x, pose_fb.pose.position.y, pose_fb.pose.position.z)
                
                self.path_fb.poses.append(pose_fb)
                self.path_fb.header.stamp = self.get_clock().now().to_msg()
                self.pub_path_fb.publish(self.path_fb)
            
            # 対応点の可視化（refとfbを線で結ぶ）
            if ref_pos is not None and fb_pos is not None:
                self._publish_comparison_markers(ref_pos, fb_pos)
                
        except (KeyError, ValueError) as e:
            # CSVにEE位置データがない場合は無視（古いデータとの互換性）
            pass
    
    def _publish_comparison_markers(self, ref_pos, fb_pos):
        """refとfbの対応点を線で結んで可視化"""
        marker_array = MarkerArray()
        
        # 対応点を結ぶ線
        line_marker = Marker()
        line_marker.header.frame_id = "world"
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.ns = "comparison_line"
        line_marker.id = 0
        line_marker.type = Marker.LINE_LIST
        line_marker.action = Marker.ADD
        line_marker.pose.orientation.w = 1.0
        line_marker.scale.x = 0.02  # 線の太さ
        line_marker.color.r = 1.0
        line_marker.color.g = 1.0
        line_marker.color.b = 0.0
        line_marker.color.a = 0.8
        
        from geometry_msgs.msg import Point
        # ref点
        p1 = Point()
        p1.x = ref_pos[0]
        p1.y = ref_pos[1]
        p1.z = ref_pos[2]
        line_marker.points.append(p1)
        
        # fb点
        p2 = Point()
        p2.x = fb_pos[0]
        p2.y = fb_pos[1]
        p2.z = fb_pos[2]
        line_marker.points.append(p2)
        
        marker_array.markers.append(line_marker)
        
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
        
        # 誤差ベクトルのテキスト表示
        import math
        dist = math.sqrt((ref_pos[0] - fb_pos[0])**2 + 
                        (ref_pos[1] - fb_pos[1])**2 + 
                        (ref_pos[2] - fb_pos[2])**2)
        
        text_marker = Marker()
        text_marker.header.frame_id = "world"
        text_marker.header.stamp = self.get_clock().now().to_msg()
        text_marker.ns = "comparison_text"
        text_marker.id = 3
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        # テキストを線の中間点に表示
        text_marker.pose.position.x = (ref_pos[0] + fb_pos[0]) / 2
        text_marker.pose.position.y = (ref_pos[1] + fb_pos[1]) / 2
        text_marker.pose.position.z = (ref_pos[2] + fb_pos[2]) / 2 + 0.3  # 少し上に表示
        text_marker.pose.orientation.w = 1.0
        text_marker.scale.z = 0.15  # テキストサイズ
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        text_marker.text = f"Error: {dist:.4f}m"
        
        marker_array.markers.append(text_marker)
        
        self.pub_comparison_markers.publish(marker_array)
    
    def _update_trajectory(self):
        """刃先のTFを取得して軌跡を更新（廃止：CSVから直接読み取る方式に変更）"""
        # この関数は互換性のために残すが、使用されない
        pass
    
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
