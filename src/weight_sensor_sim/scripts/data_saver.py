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

        # Subscribe to the weight sensor topics
        rospy.Subscriber('/weight_sensor/fx', Float32, self.callback_fx)
        rospy.Subscriber('/weight_sensor/fy', Float32, self.callback_fy)
        rospy.Subscriber('/weight_sensor/fz', Float32, self.callback_fz)

        # Initialize force values
        self.fx = None
        self.fy = None
        self.fz = None

        # Open CSV file in append mode
        date = datetime.now().strftime('%Y%m%d_%H:%M:%S')
        csv_filename = f'weight_sensor_data_{date}.csv'
        self.csv_file = open(csv_filename, mode='a', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        # Write header to CSV if file is empty
        self.csv_file.seek(0, 2)  # Go to the end of the file
        if self.csv_file.tell() == 0:  # Check if file is empty
            self.csv_writer.writerow(['Timestamp', 'fx', 'fy', 'fz'])

    def callback_fx(self, msg):
        self.fx = msg.data
        self.save_data()

    def callback_fy(self, msg):
        self.fy = msg.data
        self.save_data()

    def callback_fz(self, msg):
        self.fz = msg.data
        self.save_data()

    def save_data(self):
        if self.fx is not None and self.fy is not None and self.fz is not None:
            # Get current timestamp
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

            # Write timestamp and force data to CSV
            self.csv_writer.writerow([timestamp, self.fx, self.fy, self.fz])
            rospy.loginfo(f"Data saved: {timestamp}, fx: {self.fx}, fy: {self.fy}, fz: {self.fz}")

            # Reset force values to ensure new data is saved only when all three are updated again
            self.fx = None
            self.fy = None
            self.fz = None

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