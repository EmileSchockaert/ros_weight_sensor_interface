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

        # Subscribe to the weight sensor topic
        rospy.Subscriber('/weight_sensor/data', Float32, self.callback)

        # Open CSV file in append mode
        self.csv_file = open('weight_sensor_data.csv', mode='a', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        # Write header to CSV if file is empty
        self.csv_file.seek(0, 2)  # Go to the end of the file
        if self.csv_file.tell() == 0:  # Check if file is empty
            self.csv_writer.writerow(['Timestamp', 'Weight'])

    def callback(self, msg):
        # Get current timestamp
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

        # Write timestamp and weight data to CSV
        self.csv_writer.writerow([timestamp, msg.data])
        rospy.loginfo(f"Data saved: {timestamp}, {msg.data}")

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
