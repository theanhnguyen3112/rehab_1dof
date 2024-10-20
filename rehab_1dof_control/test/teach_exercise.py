#/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.srv import GetMotionPlan
import yaml

class MoveItPlanning(Node):
    def __init__(self):
        super().__init__('moveit_planning_node')

        # Tạo client cho service planning
        self.client = self.create_client(GetMotionPlan, '/plan_kinematic_path')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')

    def create_plan(self):
        # Tạo request đến service MoveIt
        req = GetMotionPlan.Request()

        # Set thông số planning (target, joints, constraints...)
        # Bạn cần cấu hình chi tiết ở đây dựa trên setup của bạn.

        # Gửi request và đợi kết quả
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            plan = future.result().motion_plan_response.trajectory
            self.save_trajectory(plan)
        else:
            self.get_logger().error('Planning failed')

    def save_trajectory(self, trajectory):
        # Lưu quỹ đạo dưới dạng YAML
        with open('trajectory.yaml', 'w') as file:
            yaml.dump(trajectory, file)
        self.get_logger().info('Trajectory saved to trajectory.yaml')

def main(args=None):
    rclpy.init(args=args)
    planning_node = MoveItPlanning()
    planning_node.create_plan()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
