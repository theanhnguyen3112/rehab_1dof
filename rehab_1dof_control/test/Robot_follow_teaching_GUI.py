#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import matplotlib.pyplot as plt
import threading
import csv
import time
from tkinter import *
from tkinter import ttk, filedialog
import pandas as pd
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# create class for node read message from /joint_states topic   
class JointStatePlotter(Node):
    def __init__(self):
        super().__init__('joint_state_plotter') # node name for read message
        self.subscription = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        
        self.time = []
        self.joint_positions = []
        self.start_time = time.time()
        
        self.is_recording = False
        self.start_time_record = 0
        self.data = []

        self.time_view = []

        self.joint_names = ['shoulder_joint']
        self.waypoints = [
            # {'positions': [1.571], 'velocities': [0.0], 'time_from_start': 0.5},
            # {'positions': [0.0], 'velocities': [0.0], 'time_from_start': 0.5},
            # {'positions': [0.7855], 'velocities': [0.0], 'time_from_start': 0.5},
            # {'positions': [0.0], 'velocities': [0.0], 'time_from_start': 0.5}
        ]
        self.current_waypoint_index = 0
        self.publisher_ = self.create_publisher(JointTrajectory, '/arm_manipulator_controller/joint_trajectory', 10)
        self.is_implementing = False

        self.time_per_publish = 0.03
        self.timer_publish_ = self.create_timer(self.time_per_publish, self.send_joint_trajectory)
    
    def send_joint_trajectory(self):
        if self.is_implementing:
            traj_msg = JointTrajectory()
            traj_msg.joint_names = self.joint_names
            waypoint = self.waypoints[self.current_waypoint_index]

            point = JointTrajectoryPoint()
            point.positions = waypoint['positions']
            point.velocities = waypoint['velocities']
            point.time_from_start = Duration(
                sec=int(waypoint['time_from_start']), 
                nanosec=int((waypoint['time_from_start'] % 1) * 1e9)
            )
            #point.time_from_start = Duration(sec=waypoint['time_from_start'])

            traj_msg.points = [point]
            self.publisher_.publish(traj_msg)
            self.get_logger().info(f'Moving to waypoint {self.current_waypoint_index + 1}: {list(point.positions)} in {point.time_from_start.sec} seconds, with velocities: {list(point.velocities)}')
            
            self.current_waypoint_index += 1
            if self.current_waypoint_index >= len(self.waypoints):
                self.current_waypoint_index = 0

    def joint_state_callback(self, msg):
        current_time = time.time() - self.start_time
        
        if 'shoulder_joint' in msg.name:
            shoulder_index = msg.name.index('shoulder_joint')

            # Chỉ giữ lại dữ liệu trong 5 giây gần nhất
            if len(self.time) > 0 and (current_time - self.time[0]) > 5:
                self.time.pop(0)
                self.joint_positions.pop(0)
            
            self.time.append(current_time)
            self.joint_positions.append(msg.position[shoulder_index])

            if self.is_recording:
                sec = msg.header.stamp.sec
                nanosec = msg.header.stamp.nanosec
                shoulder_position = msg.position[shoulder_index]
                shoulder_velocity = msg.velocity[shoulder_index]
                shoulder_effort = msg.effort[shoulder_index]
                
                self.data.append([sec, nanosec, shoulder_position, shoulder_velocity, shoulder_effort])

    # create function for "Start" button
    def start_recording(self):
        self.is_recording = True
        self.start_time_record = time.time()
        self.data = []

    # create function for "Stop" button
    def stop_recording(self):
        self.is_recording = False

    # create function for "Export" button
    def export_data(self, filename):
        with open(filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['sec', 'nanosec', 'position_shoulder','velocity_shoulder', 'effort_shoulder'])
            writer.writerows(self.data)
    
    def start_implement(self):
        self.is_implementing = True
    
    def stop_implement(self):
        self.is_implementing = False

class JointStateGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Teaching for rehabilitation robot")
        self.root.geometry('+1270+50') #610x950

        # Section 1: Real-time Joint Plot
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(6, 8))
        self.fig.tight_layout(pad=3.08)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().grid(row=0, column=0)

        # Section 2: Controls for Recording
        frame_controls = Frame(root)
        frame_controls.grid(row=1, column=0, padx=10, pady=10)

        self.btn_start = Button(frame_controls, text="Start", command=self.start_recording)
        self.btn_start.grid(row=0, column=0, padx=5)

        self.btn_stop = Button(frame_controls, text="Stop", state=DISABLED, command=self.stop_recording)
        self.btn_stop.grid(row=0, column=1, padx=5)

        self.btn_export = Button(frame_controls, text="Export", state=DISABLED, command=self.export_data)
        self.btn_export.grid(row=0, column=2, padx=5)

        self.btn_clear = Button(frame_controls, text="Clear", state=DISABLED, command=self.clear_data_recorded)
        self.btn_clear.grid(row=0, column=3, padx=5)

        # Section 3: View Recorded File
        frame_view = Frame(root)
        frame_view.grid(row=2, column=0, padx=10, pady=10)

        self.btn_import = Button(frame_view, text="Import", command=self.import_data)
        self.btn_import.grid(row=0, column=0, padx=5)

        self.combo_columns = ttk.Combobox(frame_view, values=[])
        self.combo_columns.grid(row=0, column=1, padx=5)

        self.btn_view = Button(frame_view, text="View", command=self.view_csv_plot)
        self.btn_view.grid(row=0, column=2, padx=5)

        self.btn_implement = Button(frame_view, text="Implement",state = DISABLED, command=self.start_implement)
        self.btn_implement.grid(row=0, column=3, padx=5)

        self.btn_stop_implement = Button(frame_view, text="Stop Implement",state = DISABLED, command=self.stop_implement)
        self.btn_stop_implement.grid(row=0, column=4, padx=5)

        frame_info = Frame(root)
        frame_info.grid(row=3, column=0, padx=10, pady=10)
        self.label_freq = Label(frame_info, text="", font=("Arial", 14), fg="blue")
        self.label_freq.grid(row=0, column=0, padx=5)
        self.label_state_impl = Label(frame_info, text="...", font=("Arial", 14), fg="green")
        self.label_state_impl.grid(row=0, column=2, padx=5)

        # ROS2 threading to get joint data
        self.node = JointStatePlotter()
        self.ros_thread = threading.Thread(target=self.spin_ros, daemon=True)
        self.ros_thread.start()

        # Update plot in real-time
        self.update_plot()

    def start_recording(self):
        self.node.start_recording()
        self.btn_stop.config(state=NORMAL)
        self.btn_start.config(state=DISABLED)
        self.btn_clear.config(state=NORMAL)

    def stop_recording(self):
        self.node.stop_recording()
        self.btn_stop.config(state=DISABLED)
        self.btn_export.config(state=NORMAL)

    def export_data(self):
        filename = filedialog.asksaveasfilename(defaultextension=".csv", filetypes=[("CSV files", "*.csv")])
        if filename:
            self.node.export_data(filename)

    def clear_data_recorded(self):
        self.btn_export.config(state=DISABLED)
        self.btn_clear.config(state=DISABLED)
        self.btn_stop.config(state=DISABLED)
        self.btn_start.config(state=NORMAL)
        self.node.data = []

    def start_implement(self):
        self.btn_export.config(state=DISABLED)
        self.btn_clear.config(state=DISABLED)
        self.btn_stop.config(state=DISABLED)
        self.btn_start.config(state=DISABLED)
        self.btn_implement.config(state=DISABLED)
        self.btn_stop_implement.config(state=NORMAL)
        self.label_state_impl.config(text="Implementing...")
        self.node.start_implement()
    
    def stop_implement(self):
        self.btn_export.config(state=DISABLED)
        self.btn_clear.config(state=DISABLED)
        self.btn_stop.config(state=DISABLED)
        self.btn_start.config(state=NORMAL)
        self.btn_implement.config(state=NORMAL)
        self.btn_stop_implement.config(state=DISABLED)
        self.label_state_impl.config(text="Stop implement!")
        self.node.stop_implement()

    def import_data(self):
        file_path = filedialog.askopenfilename(filetypes=[("CSV files", "*.csv")])
        if file_path:
            self.data = pd.read_csv(file_path)
            list_col = list (self.data.columns)
            self.combo_columns.config(values=list_col[2:])
            self.btn_implement.config(state = NORMAL)

            file_name = file_path.split("/")[-1]
            
            freq_Hz = 0
            freq_Hz_pre = 0
            count_different = 0
            time_col = list(self.data['sec'].to_numpy())
            first_time_sec = time_col[0]
            for i in range(1, len(time_col)):
                freq_Hz+=1
                if time_col[i] != first_time_sec:  # Kiểm tra giá trị hiện tại và giá trị trước đó
                    count_different += 1
                    freq_Hz_pre =  freq_Hz
                    freq_Hz = 0
                    first_time_sec = time_col[i]
                    if count_different == 2:
                        break
            self.label_freq.config(text = f"[{file_name}]-Frequency: {freq_Hz_pre} Hz")

            nanosec_col = list(self.data['nanosec'].to_numpy())
            pos_col = list(self.data['position_shoulder'].to_numpy())
            for i in range(len(time_col)):
                time_col_ = int(time_col[i]) +0.01*(int(nanosec_col[i])/10000000)
                self.node.time_view.append(time_col_)

                if (i%(self.node.time_per_publish*100))==0:
                    waypoint_ = {'positions': [float(pos_col[i])], 'velocities': [0.0], 'time_from_start': self.node.time_per_publish}
                    self.node.waypoints.append(waypoint_)

                # if i==0:
                #     if int((int(nanosec_col[i])/10000000))>=50:
                #         time_from_start_= 1.0 - 0.01*(int(nanosec_col[i])/10000000)
                #     else:
                #         time_from_start_= 0.5 - 0.01*(int(nanosec_col[i])/10000000)
                #     waypoint_ = {'positions': [float(pos_col[i])], 'velocities': [0.0], 'time_from_start': time_from_start_}
                #     self.node.waypoints.append(waypoint_)
                # elif i==(len(time_col)-1):
                #     if int((int(nanosec_col[i])/10000000))>50:
                #         time_from_start_= 0.01*(int(nanosec_col[i])/10000000)-0.5
                #     else:
                #         time_from_start_= 0.01*(int(nanosec_col[i])/10000000)
                #     time_from_start_=0.01*(int(nanosec_col[i])/10000000)
                #     waypoint_ = {'positions': [float(pos_col[i])], 'velocities': [0.0], 'time_from_start': time_from_start_}
                #     self.node.waypoints.append(waypoint_)
                # elif int(int(nanosec_col[i])/10000000)==0:# or int(int(nanosec_col[i])/10000000)==50:
                #     waypoint_ = {'positions': [float(pos_col[i])], 'velocities': [0.0], 'time_from_start': 0.5}
                #     self.node.waypoints.append(waypoint_)

            # self.time_view = [round(i * (1/freq_Hz_pre), 2) for i in range(len(time_col))]

    def view_csv_plot(self):
        column = self.combo_columns.get()
        if column:
            self.ax2.clear()  # Clear the previous plot
            self.ax2.plot(self.node.time_view, self.data[column].to_numpy())
            self.ax2.set_xlabel('Time (sec)')
            self.ax2.set_ylabel(column)
            self.ax2.set_title(f"{column} over Time")
            self.ax2.grid(True)
            self.ax2.set_ylim(-0.1, 1.7)
            self.ax2.set_autoscaley_on(False) 
            self.canvas.draw()  # Update the canvas

    def spin_ros(self):
        rclpy.spin(self.node)

    def update_plot(self):
        if len(self.node.time) > 1:
            self.ax1.clear()
            self.ax1.plot(self.node.time, self.node.joint_positions, label='Shoulder Joint Angle')
            self.ax1.set_xlabel('Time (s)')
            self.ax1.set_ylabel('Joint Angle')
            self.ax1.set_title('Real-time Joint Angles (Last 5 seconds)')
            self.ax1.legend()
            self.ax1.grid(True)
            self.ax1.set_ylim(-0.1, 1.7)
            self.ax1.set_autoscaley_on(False) 
            self.canvas.draw()

        self.root.after(100, self.update_plot)

def main():
    rclpy.init()
    root = Tk()
    gui = JointStateGUI(root)
    root.mainloop()
    rclpy.shutdown()

if __name__ == "__main__":
    main()