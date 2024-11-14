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

        self.x_data = 0 # Initial position of the indicator arrow
        self.current_position = 0 # Current position of the indicator arrow
        self.target_position = 0 # Target position of the indicator arrow
        
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

        # Set up ROS subscriber
        rospy.Subscriber('/weight_sensor/data', Float32, self.callback)

        # Start updating the indicator
        self.update_indicator()

    def callback(self, msg):
        self.x_data = msg.data
        # Map force value to 0-255 for color bar positioning
        self.target_position = max(0, min(255, int((self.x_data / 8) * 255)))

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