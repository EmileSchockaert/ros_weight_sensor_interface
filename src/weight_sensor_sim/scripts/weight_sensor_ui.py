#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
from collections import deque

class WeightSensorUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Weight Sensor Data")
        
        # Set up the matplotlib figure and deque for storing data
        self.fig, self.ax = plt.subplots()
        self.x_data = deque(maxlen=10)  # Stores the time points
        self.y_data = deque(maxlen=10)  # Stores the weight data
        
        # Create the canvas for matplotlib and add it to the tkinter window
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().pack()
        
        # Set up ROS subscriber
        rospy.Subscriber('/weight_sensor/data', Float32, self.callback)
        
        # Start updating the plot
        self.update_plot()

    def callback(self, msg):
        # Append the received weight data to the deque
        self.y_data.append(msg.data)
        # Append the time (or index) to the x-axis data
        if len(self.x_data) == 0:
            self.x_data.append(0)
        else:
            self.x_data.append(self.x_data[-1] + 0.5)

    def update_plot(self):        
        # Clear and redraw the plot
        self.ax.clear()
        self.ax.plot(self.x_data, self.y_data, label="Weight")
        self.ax.set_ylim(0, 8)  # Adjust based on expected range
        self.ax.set_title("Real-Time Weight Sensor Data")
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Force (N)")
        self.ax.legend(loc="upper right")
        
        # Redraw the canvas
        self.canvas.draw()
        
        # Schedule the update every 500 ms
        self.root.after(500, self.update_plot)

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('weight_sensor_ui', anonymous=True)
    
    # Set up the Tkinter root window and pass it to the WeightSensorUI class
    root = tk.Tk()
    app = WeightSensorUI(root)
    
    # Start the Tkinter main loop
    try:
        root.mainloop()
    except rospy.ROSInterruptException:
        pass
