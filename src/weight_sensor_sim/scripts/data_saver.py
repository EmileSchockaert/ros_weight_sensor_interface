#!/usr/bin/env python

# run in terminal with:
#   rosrun weight_sensor_sim data_saver.py

import rospy
from std_msgs.msg import Float32
import csv
from datetime import datetime

class DataSaver:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('data_saver', anonymous=True)

        # Subscribe to the weight sensor topics for sensor 1
        rospy.Subscriber('/weight_sensor1/fx', Float32, self.callback_fx1)
        rospy.Subscriber('/weight_sensor1/fy', Float32, self.callback_fy1)
        rospy.Subscriber('/weight_sensor1/fz', Float32, self.callback_fz1)

        # Subscribe to the weight sensor topics for sensor 2
        rospy.Subscriber('/weight_sensor2/fx', Float32, self.callback_fx2)
        rospy.Subscriber('/weight_sensor2/fy', Float32, self.callback_fy2)
        rospy.Subscriber('/weight_sensor2/fz', Float32, self.callback_fz2)

        # Initialize force values for sensor 1
        self.fx1 = None
        self.fy1 = None
        self.fz1 = None

        # Initialize force values for sensor 2
        self.fx2 = None
        self.fy2 = None
        self.fz2 = None

        # Open CSV file in append mode
        date = datetime.now().strftime('%Y%m%d_%H:%M:%S')
        csv_filename = f'weight_sensor_data_{date}.csv'
        self.csv_file = open(csv_filename, mode='a', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        # Write header to CSV if file is empty
        self.csv_file.seek(0, 2)  # Go to the end of the file
        if self.csv_file.tell() == 0:  # Check if file is empty
            self.csv_writer.writerow(['Timestamp', 'fx1', 'fy1', 'fz1', 'fx2', 'fy2', 'fz2'])

    def callback_fx1(self, msg):
        self.fx1 = msg.data
        self.save_data()

    def callback_fy1(self, msg):
        self.fy1 = msg.data
        self.save_data()

    def callback_fz1(self, msg):
        self.fz1 = msg.data
        self.save_data()

    def callback_fx2(self, msg):
        self.fx2 = msg.data
        self.save_data()

    def callback_fy2(self, msg):
        self.fy2 = msg.data
        self.save_data()

    def callback_fz2(self, msg):
        self.fz2 = msg.data
        self.save_data()

    def save_data(self):
        if (self.fx1 is not None and self.fy1 is not None and self.fz1 is not None and
            self.fx2 is not None and self.fy2 is not None and self.fz2 is not None):
            # Get current timestamp
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

            # Write timestamp and force data to CSV
            self.csv_writer.writerow([timestamp, self.fx1, self.fy1, self.fz1, self.fx2, self.fy2, self.fz2])
            rospy.loginfo(f"Data saved: {timestamp}, fx1: {self.fx1}, fy1: {self.fy1}, fz1: {self.fz1}, fx2: {self.fx2}, fy2: {self.fy2}, fz2: {self.fz2}")

            # Reset force values to ensure new data is saved only when all six are updated again
            self.fx1 = None
            self.fy1 = None
            self.fz1 = None
            self.fx2 = None
            self.fy2 = None
            self.fz2 = None

    def shutdown(self):
        # Close the CSV file on shutdown
        self.csv_file.close()
        rospy.loginfo("CSV file closed.")

if __name__ == '__main__':
    try:
        data_saver = DataSaver()
        rospy.on_shutdown(data_saver.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass