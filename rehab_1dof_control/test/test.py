#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tkinter as tk
from threading import Thread

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello, world! {self.count}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.count += 1

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(String, 'topic', self.listener_callback, 10)

    def listener_callback(self, msg):
        print(f'Received: "{msg.data}"')

def start_ros():
    rclpy.init()
    publisher_node = PublisherNode()
    subscriber_node = SubscriberNode()
    
    while rclpy.ok():
        rclpy.spin_once(publisher_node)
        rclpy.spin_once(subscriber_node)

def start_gui():
    root = tk.Tk()
    root.title("ROS2 GUI")
    
    label = tk.Label(root, text="ROS2 Node Running")
    label.pack()

    button = tk.Button(root, text="Exit", command=root.quit)
    button.pack()

    root.mainloop()

if __name__ == "__main__":
    ros_thread = Thread(target=start_ros)
    ros_thread.start()
    
    start_gui()
    
    ros_thread.join()

