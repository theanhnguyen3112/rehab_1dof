#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration  # Thêm Duration để định nghĩa thời gian

class RobotJointController(Node):
    def __init__(self):
        super().__init__('robot_joint_controller')
        
        # Tạo publisher gửi đến topic '/joint_trajectory_controller/joint_trajectory'
        self.publisher_ = self.create_publisher(JointTrajectory, '/arm_manipulator_controller/joint_trajectory', 10)
        
        # Khởi tạo bộ đếm thời gian để gửi lệnh điều khiển mỗi 5 giây
        self.timer = self.create_timer(3.0, self.send_joint_trajectory)

        # Danh sách tên các khớp của robot
        self.joint_names = ['shoulder_joint']

        # Tạo danh sách các waypoint với các vị trí và thời gian khác nhau
        self.waypoints = [
            {'positions': [1.571], 'velocities': [0.0], 'time_from_start': 2},  # Waypoint 1: Về vị trí gốc trong 2 giây
            {'positions': [0.0], 'velocities': [0.0], 'time_from_start': 2},  # Waypoint 2: Đến vị trí (0.5, -0.5) trong 3 giây
            {'positions': [0.7855], 'velocities': [0.0], 'time_from_start': 2},  # Waypoint 3: Đến vị trí (-0.5, 0.5) trong 4 giây
            {'positions': [0.0], 'velocities': [0.0], 'time_from_start': 2}  # Waypoint 4: Đến vị trí (0.25, -0.25) trong 5 giây
        ]

        # Biến đếm để theo dõi waypoint hiện tại
        self.current_waypoint_index = 0

    def send_joint_trajectory(self):
        # Khởi tạo thông điệp JointTrajectory
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names

        # Lấy waypoint hiện tại
        waypoint = self.waypoints[self.current_waypoint_index]

        # Tạo điểm JointTrajectoryPoint và đặt vị trí và thời gian từ waypoint
        point = JointTrajectoryPoint()
        point.positions = waypoint['positions']
        point.velocities = waypoint['velocities']
        point.time_from_start = Duration(sec=waypoint['time_from_start'])

        # Thêm điểm vào thông điệp JointTrajectory
        traj_msg.points = [point]

        # Gửi thông điệp JointTrajectory
        self.publisher_.publish(traj_msg)

        # In ra console để theo dõi
        self.get_logger().info(f'Moving to waypoint {self.current_waypoint_index + 1}: {list(point.positions)} in {point.time_from_start.sec} seconds, with velocities:{list(point.velocities)}')

        # Tăng biến đếm waypoint
        self.current_waypoint_index += 1

        # Nếu đã đi hết các waypoint, quay lại waypoint đầu tiên (lặp lại vô hạn)
        if self.current_waypoint_index >= len(self.waypoints):
            self.current_waypoint_index = 0

def main(args=None):
    rclpy.init(args=args)
    robot_joint_controller = RobotJointController()
    rclpy.spin(robot_joint_controller)
    
    # Kết thúc node
    robot_joint_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
