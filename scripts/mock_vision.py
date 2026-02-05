#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros

class MockVisionNode(Node):
    def __init__(self):
        super().__init__('mock_vision_node')
        # 创建 TF 广播器
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        # 10Hz 频率发布
        self.timer = self.create_timer(0.1, self.broadcast_timer_callback)
        self.get_logger().info("模拟识别节点已启动！")
        self.get_logger().info("正在发布相对于相机的目标物：target_box")

    def broadcast_timer_callback(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        
        # 关键点：这是相对于相机的视觉坐标系
        t.header.frame_id = 'camera_color_optical_frame'
        t.child_frame_id = 'target_box'
        
        # 模拟物体位置：在相机正前方 20cm (Z轴)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.2
        
        # 无旋转
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = MockVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()