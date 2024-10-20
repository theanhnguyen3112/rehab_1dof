#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
import time

class JointStatePlotter(Node):
    def __init__(self):
        super().__init__('joint_state_plotter')
        
        # Subscribe to the joint states topic
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        
        # Initialize lists to store time and joint positions
        self.time = []
        self.joint_1_positions = []
        self.timetrc = [1]
        self.dain = True

        # Start time for tracking 5 seconds duration
        self.start_time = time.time()

    def joint_state_callback(self, msg):
        # Get the current time
        current_time = time.time() - self.start_time

        # Append the current time and positions of the joints to the lists
        self.time.append(current_time)

        # Assume the first and second joints correspond to 'shoulder_joint' and 'elbow_joint'
        if 'shoulder_joint' in msg.name:
            shoulder_index = msg.name.index('shoulder_joint')

            # Append joint positions to the lists
            self.joint_1_positions.append(msg.position[shoulder_index])

            self.timetrc.append(time.time())

            if float(msg.position[shoulder_index]) < 0.02:
                self.timetrc.pop(0)
            
            if len(self.timetrc) == 2:
                self.dain==True

            if -0.02< float(msg.position[shoulder_index])-0.785 < 0.02 and self.dain==False:
                self.get_logger().info(f'time: ')
                self.dain=True
                

        # Remove data points that are older than 5 seconds
        while self.time and (current_time - self.time[0]) > 10:
            self.time.pop(0)
            self.joint_1_positions.pop(0)

        # Print current joint states for verification
        #self.get_logger().info(f'Shoulder Joint: {self.joint_1_positions[-1]}, Elbow Joint: {self.joint_2_positions[-1]}')

        # Optional: Plot in real-time
        if len(self.time) > 1:
            plt.clf()
            plt.plot(self.time, self.joint_1_positions, label='Shoulder Joint Angle')
            plt.xlabel('Time (s)')
            plt.ylabel('Joint Angle')
            plt.title('Joint Angles Over Last 5 Seconds')
            plt.legend()
            plt.grid(True)
            plt.pause(0.1)  # Update the plot every 0.1 seconds

    def plot_data(self):
        # Plot the data after gathering sufficient points
        plt.figure()
        plt.plot(self.time, self.joint_1_positions, label='Shoulder Joint Angle')
        plt.xlabel('Time (s)')
        plt.ylabel('Joint Angle')
        plt.xlim((-0.1, 1.7))
        plt.title('Joint Angles Over Last 5 Seconds')
        plt.legend()
        plt.grid(True)
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = JointStatePlotter()

    # Run the node until interrupted
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # After shutting down the node, plot the data collected
    node.plot_data()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
