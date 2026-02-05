#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class VPGPerceptionNode(Node):
    def __init__(self):
        super().__init__('vpg_perception_node')

        # 初始化 TF 监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 创建一个定时器，每秒计算一次坐标转换
        self.timer = self.create_timer(1.0, self.on_timer)
        self.get_logger().info("坐标转换监听节点已启动，正在计算 target_box -> base_link...")

    def on_timer(self):
        # 定义我们想要转换的两个坐标系
        target_frame = 'target_box'
        base_frame = 'base_link'

        try:
            # 关键代码：从 TF 树中查找 target_box 相对于 base_link 的位置
            # time=0 表示获取最新的可用变换
            now = rclpy.time.Time()
            t = self.tf_buffer.lookup_transform(
                base_frame,
                target_frame,
                now)

            # 提取坐标数值
            x = t.transform.translation.x
            y = t.transform.translation.y
            z = t.transform.translation.z

            self.get_logger().info(f'【物块位置已锁定】在机器人基座坐标系下: X={x:.3f}, Y={y:.3f}, Z={z:.3f}')

        except TransformException as ex:
            self.get_logger().info(f'无法计算坐标转换: {ex}')

def main():
    rclpy.init()
    node = VPGPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()