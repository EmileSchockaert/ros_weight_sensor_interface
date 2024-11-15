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

        self.fx = 0
        self.fy = 0
        self.fz = 0
        self.current_position = 0
        self.target_position = 0

        # Create a figure for the color gradient
        self.fig, self.ax = plt.subplots(figsize=(8, 2))
        self.fig.subplots_adjust(bottom=0.5, top=0.8, left=0.1, right=0.9)
        
        # Create gradient color bar from green to red
        gradient = np.linspace(0, 1, 256)
        gradient = np.vstack((gradient, gradient))
        self.ax.imshow(gradient, aspect='auto', cmap="RdYlGn_r")
        
        # Set x-ticks and labels
        self.ax.set_xticks([0, 64, 128, 191, 255])
        self.ax.set_xticklabels(['0 Newton', '2 Newton', '4 Newton', '6 Newton', 'High Risk'])
        self.ax.get_yaxis().set_visible(False)
        
        # Create an arrow indicator
        self.indicator = self.ax.axvline(x=0, color='black', linewidth=2)

        # Add the color bar to tkinter using FigureCanvasTkAgg
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().pack()

        # Set up ROS subscribers
        rospy.Subscriber('/weight_sensor/fx', Float32, self.callback_fx)
        rospy.Subscriber('/weight_sensor/fy', Float32, self.callback_fy)
        rospy.Subscriber('/weight_sensor/fz', Float32, self.callback_fz)

        # Start updating the indicator
        self.update_indicator()

    def callback_fx(self, msg):
        self.fx = msg.data
        self.update_target_position()

    def callback_fy(self, msg):
        self.fy = msg.data
        self.update_target_position()

    def callback_fz(self, msg):
        self.fz = msg.data
        self.update_target_position()

    def calculate_total_force(self):
        return np.power(self.fx**2 + self.fy**2 + self.fz**2, 0.5)

    def update_target_position(self):
        total_force = self.calculate_total_force()
        # Map total force value to 0-255 for color bar positioning
        self.target_position = max(0, min(255, int((total_force / 8) * 255)))

    def update_indicator(self):
        # Smoothly move the current position towards the target position
        step_size = 20  # Smaller step size for smoother movement
        if self.current_position < self.target_position:
            self.current_position += min(step_size, self.target_position - self.current_position)
        elif self.current_position > self.target_position:
            self.current_position -= min(step_size, self.current_position - self.target_position)
        
        # Update arrow position
        self.indicator.set_xdata(self.current_position)
        
        # Redraw the canvas
        self.canvas.draw()

        # Schedule the update every 20 ms for even smoother animation
        self.root.after(50, self.update_indicator)

if __name__ == "__main__":
    rospy.init_node('force_risk_ui', anonymous=True)
    root = tk.Tk()
    app = ForceRiskUI(root)
    root.mainloop()