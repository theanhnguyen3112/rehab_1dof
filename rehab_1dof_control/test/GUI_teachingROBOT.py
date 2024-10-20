#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
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

class JointStateGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Teaching for rehabilitation robot")

        # Section 1: Real-time Joint Plot
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(12, 4))
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
        self.btn_view.grid(row=0, column=3, padx=5)

        self.label_freq = Label(frame_view, text="", font=("Arial", 14), fg="blue")
        self.label_freq.grid(row=0, column=2, padx=5)

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

    def import_data(self):
        file_path = filedialog.askopenfilename(filetypes=[("CSV files", "*.csv")])
        if file_path:
            self.data = pd.read_csv(file_path)
            list_col = list (self.data.columns)
            self.combo_columns.config(values=list_col[2:])

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

            self.time_view = [round(i * (1/freq_Hz_pre), 2) for i in range(len(time_col))]

    def view_csv_plot(self):
        column = self.combo_columns.get()
        if column:
            self.ax2.clear()  # Clear the previous plot
            self.ax2.plot(self.time_view, self.data[column].to_numpy())
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