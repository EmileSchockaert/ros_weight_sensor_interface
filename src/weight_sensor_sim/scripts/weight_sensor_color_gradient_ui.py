import rospy
from std_msgs.msg import Float32
import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import numpy as np

class ForceRiskUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Force Risk Indicator")

        self.fx1 = 0
        self.fy1 = 0
        self.fz1 = 0
        self.fx2 = 0
        self.fy2 = 0
        self.fz2 = 0
        self.current_position1 = 0
        self.target_position1 = 0
        self.current_position2 = 0
        self.target_position2 = 0

        # Create a figure for the first color gradient
        self.fig1, self.ax1 = plt.subplots(figsize=(8, 2))
        self.fig1.subplots_adjust(bottom=0.5, top=0.8, left=0.1, right=0.9)
        
        # Create gradient color bar from green to red for the first sensor
        gradient1 = np.linspace(0, 1, 256)
        gradient1 = np.vstack((gradient1, gradient1))
        self.ax1.imshow(gradient1, aspect='auto', cmap="RdYlGn_r")
        
        # Set x-ticks and labels for the first sensor
        self.ax1.set_xticks([0, 64, 128, 191, 255])
        self.ax1.set_xticklabels(['0 Newton', '2 Newton', '4 Newton', '6 Newton', 'High Risk'])
        self.ax1.get_yaxis().set_visible(False)
        
        # Create an arrow indicator for the first sensor
        self.indicator1 = self.ax1.axvline(x=0, color='black', linewidth=2)

        # Add the first color bar to tkinter using FigureCanvasTkAgg
        self.canvas1 = FigureCanvasTkAgg(self.fig1, master=self.root)
        self.canvas1.get_tk_widget().pack()

        # Create a figure for the second color gradient
        self.fig2, self.ax2 = plt.subplots(figsize=(8, 2))
        self.fig2.subplots_adjust(bottom=0.5, top=0.8, left=0.1, right=0.9)
        
        # Create gradient color bar from green to red for the second sensor
        gradient2 = np.linspace(0, 1, 256)
        gradient2 = np.vstack((gradient2, gradient2))
        self.ax2.imshow(gradient2, aspect='auto', cmap="RdYlGn_r")
        
        # Set x-ticks and labels for the second sensor
        self.ax2.set_xticks([0, 64, 128, 191, 255])
        self.ax2.set_xticklabels(['0 Newton', '2 Newton', '4 Newton', '6 Newton', 'High Risk'])
        self.ax2.get_yaxis().set_visible(False)
        
        # Create an arrow indicator for the second sensor
        self.indicator2 = self.ax2.axvline(x=0, color='black', linewidth=2)

        # Add the second color bar to tkinter using FigureCanvasTkAgg
        self.canvas2 = FigureCanvasTkAgg(self.fig2, master=self.root)
        self.canvas2.get_tk_widget().pack()

        # Set up ROS subscribers for the first sensor
        rospy.Subscriber('/weight_sensor1/fx', Float32, self.callback_fx1)
        rospy.Subscriber('/weight_sensor1/fy', Float32, self.callback_fy1)
        rospy.Subscriber('/weight_sensor1/fz', Float32, self.callback_fz1)

        # Set up ROS subscribers for the second sensor
        rospy.Subscriber('/weight_sensor2/fx', Float32, self.callback_fx2)
        rospy.Subscriber('/weight_sensor2/fy', Float32, self.callback_fy2)
        rospy.Subscriber('/weight_sensor2/fz', Float32, self.callback_fz2)

        # Start updating the indicators
        self.update_indicator()

    def callback_fx1(self, msg):
        self.fx1 = msg.data
        self.update_target_position1()

    def callback_fy1(self, msg):
        self.fy1 = msg.data
        self.update_target_position1()

    def callback_fz1(self, msg):
        self.fz1 = msg.data
        self.update_target_position1()

    def callback_fx2(self, msg):
        self.fx2 = msg.data
        self.update_target_position2()

    def callback_fy2(self, msg):
        self.fy2 = msg.data
        self.update_target_position2()

    def callback_fz2(self, msg):
        self.fz2 = msg.data
        self.update_target_position2()

    def calculate_total_force1(self):
        return np.sqrt(self.fx1**2 + self.fy1**2 + self.fz1**2)

    def calculate_total_force2(self):
        return np.sqrt(self.fx2**2 + self.fy2**2 + self.fz2**2)

    def update_target_position1(self):
        total_force1 = self.calculate_total_force1()
        # Map total force value to 0-255 for color bar positioning
        self.target_position1 = max(0, min(255, int((total_force1 / 8) * 255)))

    def update_target_position2(self):
        total_force2 = self.calculate_total_force2()
        # Map total force value to 0-255 for color bar positioning
        self.target_position2 = max(0, min(255, int((total_force2 / 8) * 255)))

    def update_indicator(self):
        # Smoothly move the current position towards the target position for the first sensor
        step_size = 20  # Smaller step size for smoother movement
        if self.current_position1 < self.target_position1:
            self.current_position1 += min(step_size, self.target_position1 - self.current_position1)
        elif self.current_position1 > self.target_position1:
            self.current_position1 -= min(step_size, self.current_position1 - self.target_position1)
        
        # Update arrow position for the first sensor
        self.indicator1.set_xdata(self.current_position1)
        
        # Smoothly move the current position towards the target position for the second sensor
        if self.current_position2 < self.target_position2:
            self.current_position2 += min(step_size, self.target_position2 - self.current_position2)
        elif self.current_position2 > self.target_position2:
            self.current_position2 -= min(step_size, self.current_position2 - self.target_position2)
        
        # Update arrow position for the second sensor
        self.indicator2.set_xdata(self.current_position2)
        
        # Redraw the canvases
        self.canvas1.draw()
        self.canvas2.draw()

        # Schedule the update every 50 ms for even smoother animation
        self.root.after(50, self.update_indicator)

if __name__ == "__main__":
    rospy.init_node('force_risk_ui', anonymous=True)
    root = tk.Tk()
    app = ForceRiskUI(root)
    root.mainloop()