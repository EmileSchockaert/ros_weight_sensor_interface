#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
import random

def weight_sensor_sim():
    rospy.init_node('weight_sensor_sim', anonymous=True)
    weight_pub = rospy.Publisher('/weight_sensor/data', Float32, queue_size=10)

    rate = rospy.Rate(2)  # 2 Hz

    while not rospy.is_shutdown():
        # Simulate dummy weight data (e.g., random float in range 0 to 50)
        mu = 2  # mean
        sigma = 1  # standard deviation
        simulated_weight = random.gauss(mu, sigma)
        if simulated_weight < 0:
            simulated_weight = 0
        rospy.loginfo(f"Publishing simulated weight: {simulated_weight}")
        
        # Publish the simulated weight
        weight_pub.publish(simulated_weight)
        
        # Sleep to maintain the loop rate
        rate.sleep()

if __name__ == '__main__':
    try:
        weight_sensor_sim()
    except rospy.ROSInterruptException:
        pass