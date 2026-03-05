#!/usr/bin/env python3
"""
link_correspondence_nearest.csvを読み込んで静的に可視化するノード
Plan-Feedback間の最近傍対応点を矢印とマーカーで表示
"""
import os
import csv
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


class VisualizeCorrespondenceNode(Node):
    """Link correspondence静的可視化ノード"""
    
    def __init__(self):
        super().__init__("visualize_correspondence_node")
        
        # パラメータ
        self.declare_parameter("csv_dir", "")
        
        csv_dir = str(self.get_parameter("csv_dir").value)
        
        if not csv_dir or not os.path.exists(csv_dir):
            self.get_logger().error(f"CSV directory not found: {csv_dir}")
            raise FileNotFoundError(csv_dir)
        
        # CSVファイルパス
        self.correspondence_csv_path = os.path.join(csv_dir, "link_correspondence_nearest.csv")
        
        if not os.path.exists(self.correspondence_csv_path):
            self.get_logger().error(f"link_correspondence_nearest.csv not found: {self.correspondence_csv_path}")
            raise FileNotFoundError(self.correspondence_csv_path)
        
        # Link correspondence データ
        self.link_correspondences = {}
        self._load_link_correspondence_data()
        
        # マーカーパブリッシャー
        self.pub_markers = self.create_publisher(MarkerArray, "/correspondence_viz/markers", 10)
        
        # 定期的にマーカーを配信（1秒ごと）
        self.create_timer(1.0, self._publish_markers)
        
        self.get_logger().info("Correspondence visualization node started")
        self.get_logger().info(f"Publishing markers on topic: /correspondence_viz/markers")
    
    def _load_link_correspondence_data(self):
        """link_correspondence_nearest.csvからPlan-Feedback対応点を読み込み"""
        self.link_correspondences = {}
        
        try:
            with open(self.correspondence_csv_path, 'r') as f:
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
            else:
                self.get_logger().warn("No correspondence data found")
                
        except Exception as e:
            self.get_logger().error(f"Failed to load link correspondence data: {e}")
            import traceback
            traceback.print_exc()
            raise
    
    def _publish_markers(self):
        """Link correspondenceをマーカーとして可視化"""
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
                'body_link': (0.0, 0.5, 1.0),       # 水色
                'boom_link': (1.0, 0.5, 0.0),       # オレンジ
                'arm_link': (1.0, 1.0, 0.0),        # 黄色
                'bucket_link': (1.0, 0.0, 1.0),     # マゼンタ
                'bucket_end_link': (1.0, 0.0, 0.5), # ピンク
            }
            
            color = colors.get(link_name, (0.5, 0.5, 0.5))
            
            # Plan軌跡（点群）
            plan_points_marker = Marker()
            plan_points_marker.header.frame_id = "world"
            plan_points_marker.header.stamp = self.get_clock().now().to_msg()
            plan_points_marker.ns = f"plan_{link_name}"
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
            
            for plan_pos, fb_pos, dist in correspondences:
                p = Point()
                p.x = plan_pos[0]
                p.y = plan_pos[1]
                p.z = plan_pos[2]
                plan_points_marker.points.append(p)
            
            marker_array.markers.append(plan_points_marker)
            
            # Feedback軌跡（点群）
            fb_points_marker = Marker()
            fb_points_marker.header.frame_id = "world"
            fb_points_marker.header.stamp = self.get_clock().now().to_msg()
            fb_points_marker.ns = f"fb_{link_name}"
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
            
            # 矢印マーカーでPlan→Feedbackへの誤差ベクトルを表示（サンプリング）
            step = max(1, len(correspondences) // 50)  # 50本程度に間引き
            for i in range(0, len(correspondences), step):
                plan_pos, fb_pos, dist = correspondences[i]
                
                # 誤差が0.5mm以上ある場合のみ表示
                if dist < 0.0005:
                    continue
                
                arrow_marker = Marker()
                arrow_marker.header.frame_id = "world"
                arrow_marker.header.stamp = self.get_clock().now().to_msg()
                arrow_marker.ns = f"arrows_{link_name}"
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
                
                # 矢印のサイズ
                arrow_marker.scale.x = 0.008  # 軸の太さ
                arrow_marker.scale.y = 0.015  # 矢尻の太さ
                arrow_marker.scale.z = 0.025  # 矢尻の長さ
                
                # 色：誤差が大きいほど赤く（10cm以上で真っ赤）
                error_ratio = min(dist / 0.1, 1.0)
                arrow_marker.color.r = error_ratio
                arrow_marker.color.g = 1.0 - error_ratio
                arrow_marker.color.b = 0.0
                arrow_marker.color.a = 0.7
                
                marker_array.markers.append(arrow_marker)
            
            # 最大誤差点を強調表示（太い矢印）
            max_corr = max(correspondences, key=lambda x: x[2])
            max_plan_pos, max_fb_pos, max_dist = max_corr
            
            max_arrow_marker = Marker()
            max_arrow_marker.header.frame_id = "world"
            max_arrow_marker.header.stamp = self.get_clock().now().to_msg()
            max_arrow_marker.ns = f"max_arrow_{link_name}"
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
            
            # 統計情報テキスト
            avg_dist = sum(d for _, _, d in correspondences) / len(correspondences)
            min_dist = min(d for _, _, d in correspondences)
            
            # リンクの中心位置を計算
            center_x = sum(p[0] for p, _, _ in correspondences) / len(correspondences)
            center_y = sum(p[1] for p, _, _ in correspondences) / len(correspondences)
            center_z = sum(p[2] for p, _, _ in correspondences) / len(correspondences)
            
            stats_text_marker = Marker()
            stats_text_marker.header.frame_id = "world"
            stats_text_marker.header.stamp = self.get_clock().now().to_msg()
            stats_text_marker.ns = f"stats_{link_name}"
            stats_text_marker.id = marker_id
            marker_id += 1
            stats_text_marker.type = Marker.TEXT_VIEW_FACING
            stats_text_marker.action = Marker.ADD
            stats_text_marker.pose.position.x = center_x
            stats_text_marker.pose.position.y = center_y
            stats_text_marker.pose.position.z = center_z + 0.5
            stats_text_marker.pose.orientation.w = 1.0
            stats_text_marker.scale.z = 0.15
            stats_text_marker.color.r = 1.0
            stats_text_marker.color.g = 1.0
            stats_text_marker.color.b = 1.0
            stats_text_marker.color.a = 1.0
            stats_text_marker.text = (
                f"{link_name}\n"
                f"Points: {len(correspondences)}\n"
                f"Avg: {avg_dist*1000:.1f}mm\n"
                f"Min: {min_dist*1000:.1f}mm\n"
                f"Max: {max_dist*1000:.1f}mm"
            )
            
            marker_array.markers.append(stats_text_marker)
        
        self.pub_markers.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = VisualizeCorrespondenceNode()
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
