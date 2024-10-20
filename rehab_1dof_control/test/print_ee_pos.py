#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped

class TF2Echo(Node):
    def __init__(self):
        super().__init__('tf2_echo')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Source and target frames
        self.source_frame = 'world'
        self.target_frame = 'hand_link'

        # Call the timer every 1 second to check the transform
        self.timer = self.create_timer(0.1, self.get_transform)

    def get_transform(self):
        try:
            now = rclpy.time.Time()
            # Lookup transform between source_frame and target_frame
            trans: TransformStamped = self.tf_buffer.lookup_transform(self.source_frame, self.target_frame, now)
            #self.get_logger().info(f"Transform: {trans}")

            self.get_logger().info(f"Translation: {trans.transform.translation}")
            #self.get_logger().info(f"Rotation: {trans.transform.rotation}")
        except Exception as ex:
            self.get_logger().warn(f"Could not find transform: {ex}")

def main(args=None):
    rclpy.init(args=args)
    node = TF2Echo()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

