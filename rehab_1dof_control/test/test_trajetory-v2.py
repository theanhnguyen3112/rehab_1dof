#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import matplotlib.pyplot as plt

class RobotJointController(Node):
    def __init__(self):
        super().__init__('robot_joint_controller')
        
        self.publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.timer = self.create_timer(3.0, self.send_joint_trajectory)
 
        self.joint_names = ['shoulder_joint', 'elbow_joint']

        self.waypoints = [
            {'positions': [0.0, 0.0], 'time_from_start': 2},
            {'positions': [0.785, 0.0], 'time_from_start': 2},
            {'positions': [0.0, 0.0], 'time_from_start': 2},
            {'positions': [-0.785, 0.0], 'time_from_start': 2}
        ]

        self.current_waypoint_index = 0

        # List để lưu trữ trạng thái các khớp (vị trí, vận tốc, thời gian)
        self.positions_data = {name: [] for name in self.joint_names}
        self.time_data = []  # Lưu thời gian tương ứng

    def send_joint_trajectory(self):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names

        waypoint = self.waypoints[self.current_waypoint_index]

        point = JointTrajectoryPoint()
        point.positions = waypoint['positions']
        point.time_from_start = Duration(sec=waypoint['time_from_start'])

        traj_msg.points = [point]

        self.publisher_.publish(traj_msg)

        # In ra console để theo dõi
        self.get_logger().info(f'Moving to waypoint {self.current_waypoint_index + 1}: {list(point.positions)} in {point.time_from_start.sec} seconds, with velocities:{list(point.velocities)}')

        # Ghi lại dữ liệu
        self.time_data.append(self.current_waypoint_index * 3)  # Giả định mỗi lần gửi lệnh cách nhau 3 giây
        for i, name in enumerate(self.joint_names):
            self.positions_data[name].append(point.positions[i])

        self.current_waypoint_index += 1
        if self.current_waypoint_index >= len(self.waypoints):
            self.current_waypoint_index = 0
            self.plot_joint_states()  # Vẽ đồ thị sau khi hoàn thành một chu trình

    def plot_joint_states(self):
        # Vẽ đồ thị vị trí các khớp
        plt.figure()
        for name in self.joint_names:
            plt.plot(self.time_data, self.positions_data[name], label=f'{name} position')
        plt.xlabel('Time (s)')
        plt.ylabel('Joint Positions (rad)')
        plt.title('Joint Positions over Time')
        plt.legend()
        plt.grid(True)
        plt.show()

        # Vẽ đồ thị vận tốc các khớp
        plt.figure()
        for name in self.joint_names:
            plt.plot(self.time_data, self.velocities_data[name], label=f'{name} velocity')
        plt.xlabel('Time (s)')
        plt.ylabel('Joint Velocities (rad/s)')
        plt.title('Joint Velocities over Time')
        plt.legend()
        plt.grid(True)
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    robot_joint_controller = RobotJointController()
    rclpy.spin(robot_joint_controller)
    
    robot_joint_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
