#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import threading

class TF2Echo(Node):
    def __init__(self):
        super().__init__('tf2_echo')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Source and target frames
        self.source_frame = 'world'
        self.target_frame = 'hand_link'

        # Variable for save data
        self.x_data = []
        self.y_data = []
        self.z_data = []
        self.data_lock = threading.Lock()  # Khóa để đảm bảo an toàn cho dữ liệu

        # Call the timer every 0.1 seconds to check the transform
        self.timer = self.create_timer(0.1, self.get_transform)

    def get_transform(self):
        try:
            now = rclpy.time.Time()
            # Lookup transform between source_frame and target_frame
            trans: TransformStamped = self.tf_buffer.lookup_transform(self.source_frame, self.target_frame, now)

            # Cập nhật dữ liệu với khóa
            with self.data_lock:
                self.x_data.append(trans.transform.translation.x)
                self.y_data.append(trans.transform.translation.y)
                self.z_data.append(trans.transform.translation.z)

            self.get_logger().info(f"Translation: list({trans.transform.translation})")
        except Exception as ex:
            self.get_logger().warn(f"Could not find transform: {ex}")

# Hàm để vẽ đồ thị 3D
def plot_graph(x_data, y_data, z_data, data_lock):
    # Thiết lập đồ thị
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')

    plt.gca().set_xticks([])
    plt.gca().set_yticks([])
    plt.gca().set_zticks([])

    # Hàm cập nhật cho hoạt ảnh
    def update(frame):
        with data_lock:
            ax.cla()  # Xóa đồ thị cũ
            ax.set_xlabel('X [m]')
            ax.set_ylabel('Y [m]')
            ax.set_zlabel('Z [m]')
            ax.scatter(x_data, y_data, z_data, c='r', marker='o', linewidths=0.1)  # Vẽ lại các điểm

    # Tạo hoạt ảnh
    ani = FuncAnimation(fig, update, frames=None, interval=100)

    # Hiện thị đồ thị
    plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = TF2Echo()
    
    # Bắt đầu luồng vẽ đồ thị
    thread = threading.Thread(target=plot_graph, args=(node.x_data, node.y_data, node.z_data, node.data_lock))
    thread.daemon = True
    thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
