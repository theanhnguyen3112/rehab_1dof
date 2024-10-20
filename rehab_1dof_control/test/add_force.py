#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench, Vector3

class WrenchPublisher(Node):
    def __init__(self):
        super().__init__('wrench_publisher')

        # Tạo publisher để xuất thông điệp Wrench
        self.publisher_ = self.create_publisher(Wrench, '/rehab_1dof/shoulder_control_wrench', 10)

        # Gửi wrench cứ sau 0.5 giây
        self.timer = self.create_timer(1.0, self.publish_wrench)

    def publish_wrench(self):
        # Tạo thông điệp Wrench
        wrench = Wrench()

        # Áp dụng lực (force)
        wrench.force = Vector3(x=0.0, y=0.0, z=0.0)  # Lực theo trục X

        # Áp dụng torque (mô-men xoắn)
        wrench.torque = Vector3(x=0.0, y=0.0, z=0.0)  # Mô-men xoắn quanh trục Z

        # Xuất thông điệp
        self.publisher_.publish(wrench)
        self.get_logger().info(f'Publishing Wrench: force={wrench.force}, torque={wrench.torque}')

def main(args=None):
    rclpy.init(args=args)

    node = WrenchPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
