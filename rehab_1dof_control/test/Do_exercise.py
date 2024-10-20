#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import tkinter as tk
from threading import Thread

class RobotJointController(Node):
    def __init__(self):
        super().__init__('robot_joint_controller')
        self.publisher_ = self.create_publisher(JointTrajectory, '/arm_manipulator_controller/joint_trajectory', 10)
        self.timer = self.create_timer(3.0, self.send_joint_trajectory)
        self.joint_names = ['shoulder_joint']
        self.waypoints = [
            {'positions': [1.571], 'velocities': [0.0], 'time_from_start': 2},
            {'positions': [0.0], 'velocities': [0.0], 'time_from_start': 2},
            {'positions': [0.7855], 'velocities': [0.0], 'time_from_start': 2},
            {'positions': [0.0], 'velocities': [0.0], 'time_from_start': 2}
        ]
        self.current_waypoint_index = 0

    def send_joint_trajectory(self):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names
        waypoint = self.waypoints[self.current_waypoint_index]
        point = JointTrajectoryPoint()
        point.positions = waypoint['positions']
        point.velocities = waypoint['velocities']
        point.time_from_start = Duration(sec=waypoint['time_from_start'])
        traj_msg.points = [point]
        self.publisher_.publish(traj_msg)
        self.get_logger().info(f'Moving to waypoint {self.current_waypoint_index + 1}: {list(point.positions)} in {point.time_from_start.sec} seconds, with velocities: {list(point.velocities)}')
        self.current_waypoint_index += 1
        if self.current_waypoint_index >= len(self.waypoints):
            self.current_waypoint_index = 0

def run_robot_controller():
    rclpy.init()
    robot_joint_controller = RobotJointController()
    rclpy.spin(robot_joint_controller)
    robot_joint_controller.destroy_node()
    rclpy.shutdown()

def start_robot_controller_thread():
    thread = Thread(target=run_robot_controller)
    thread.start()

def create_gui():
    root = tk.Tk()
    root.title("Robot Joint Controller")

    start_button = tk.Button(root, text="Start Robot", command=start_robot_controller_thread)
    start_button.pack(pady=20)

    root.mainloop()

if __name__ == '__main__':
    create_gui()
