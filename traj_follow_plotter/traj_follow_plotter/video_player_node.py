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
        
        csv_path = str(self.get_parameter("csv_path").value)
        playback_speed = float(self.get_parameter("playback_speed").value)
        self.loop = bool(self.get_parameter("loop").value)
        start_delay = float(self.get_parameter("start_delay").value)
        
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
        
        # 軌跡データ（刃先の位置履歴）
        self.path_ref = Path()
        self.path_ref.header.frame_id = "ref/base_link"
        self.path_fb = Path()
        self.path_fb.header.frame_id = "fb/base_link"
        
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
    
    def publish_frame(self):
        """1フレーム分のJointStateを配信"""
        if self.frame_idx >= len(self.data):
            if self.loop:
                self.frame_idx = 0
                # ループ時にPathをリセット
                self.path_ref.poses.clear()
                self.path_fb.poses.clear()
                self.get_logger().info("Looping playback... (Path reset)")
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
        
        # 刃先の位置を取得して軌跡に追加（少し待ってからTF取得）
        # one_shotがないので、コールバック内でキャンセルする方式
        def delayed_update():
            self._update_trajectory()
            # このタイマーは一度だけ実行されるようにキャンセル
            if hasattr(self, '_trajectory_timer') and self._trajectory_timer is not None:
                self._trajectory_timer.cancel()
        
        if hasattr(self, '_trajectory_timer') and self._trajectory_timer is not None:
            self._trajectory_timer.cancel()
        self._trajectory_timer = self.create_timer(0.01, delayed_update)
        
        self.frame_idx += 1
    
    def _update_trajectory(self):
        """刃先のTFを取得して軌跡を更新"""
        try:
            # まずは名前空間なしで試す（robot_state_publisherの動作による）
            try:
                trans_ref = self.tf_buffer.lookup_transform(
                    'base_link',
                    'bucket_end_link',
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.05)
                )
                trans_fb = self.tf_buffer.lookup_transform(
                    'base_link',
                    'bucket_end_link',
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.05)
                )
            except TransformException:
                # 名前空間付きで試す
                trans_ref = self.tf_buffer.lookup_transform(
                    'ref/base_link',
                    'ref/bucket_end_link',
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.05)
                )
                trans_fb = self.tf_buffer.lookup_transform(
                    'fb/base_link',
                    'fb/bucket_end_link',
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.05)
                )
            
            pose_ref = PoseStamped()
            pose_ref.header.stamp = self.get_clock().now().to_msg()
            pose_ref.header.frame_id = "ref/base_link"
            pose_ref.pose.position.x = trans_ref.transform.translation.x
            pose_ref.pose.position.y = trans_ref.transform.translation.y
            pose_ref.pose.position.z = trans_ref.transform.translation.z
            pose_ref.pose.orientation = trans_ref.transform.rotation
            
            self.path_ref.poses.append(pose_ref)
            self.path_ref.header.stamp = self.get_clock().now().to_msg()
            
            pose_fb = PoseStamped()
            pose_fb.header.stamp = self.get_clock().now().to_msg()
            pose_fb.header.frame_id = "fb/base_link"
            pose_fb.pose.position.x = trans_fb.transform.translation.x
            pose_fb.pose.position.y = trans_fb.transform.translation.y
            pose_fb.pose.position.z = trans_fb.transform.translation.z
            pose_fb.pose.orientation = trans_fb.transform.rotation
            
            self.path_fb.poses.append(pose_fb)
            self.path_fb.header.stamp = self.get_clock().now().to_msg()
            
            # 軌跡を配信
            self.pub_path_ref.publish(self.path_ref)
            self.pub_path_fb.publish(self.path_fb)
            
        except TransformException as ex:
            # 初回はTFがまだ準備できていない可能性があるので無視
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
