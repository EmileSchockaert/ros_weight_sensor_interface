#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
import random

class WeightSensorSim:
    def __init__(self) -> None:
        rospy.init_node('weight_sensor_sim', anonymous=True)

        self.weight_pubs = {
            'fx': rospy.Publisher('/weight_sensor/fx', Float32, queue_size=10),
            'fy': rospy.Publisher('/weight_sensor/fy', Float32, queue_size=10),
            'fz': rospy.Publisher('/weight_sensor/fz', Float32, queue_size=10),
        }

        self.rate = rospy.Rate(2)  # 2 Hz

    def simulate_weight_forces(self):
        while not rospy.is_shutdown():
            # Simulate dummy weight data
            mu = 2  # mean
            sigma = 1  # standard deviation
            
            for force, weight_pub in self.weight_pubs.items():

                simulated_weight = random.gauss(mu, sigma)
                if simulated_weight < 0:
                    simulated_weight = 0
                weight_pub.publish(simulated_weight)
                
                rospy.loginfo(f"Sensor weight: {force} = {simulated_weight:.2f} Newtons")
            
            # Sleep to maintain the loop rate
            self.rate.sleep()

if __name__ == '__main__':
    try:
        weight_sensor_sim = WeightSensorSim()
        weight_sensor_sim.simulate_weight_forces()
    except rospy.ROSInterruptException:
        pass