#!/usr/bin/env python
import rospy
import time
import copy
from enum import Enum
from math import sqrt, pow
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point
import math


class SimpleMovement:

    # Node initialization
    def __init__(self):

        # Create the publisher and subscriber
        self.position_pub = rospy.Publisher('/uav/input/position', Vector3, queue_size=1)
        self.cur_pos = Vector3()
        self.cur_pos.x = 0
        self.cur_pos.y = 0

        self.oscillation_rate = 0.05
        self.oscillation_amplitude = 1
        self.oscillation_mean = 3

        self.cur_pos.z = self.oscillation_mean
        time.sleep(10)  # let it boot up
        self.mainloop()

    # The main loop of the function
    def mainloop(self):
        # Set the rate of this loop
        rate = rospy.Rate(1)  # 20

        # While ROS is still running
        while not rospy.is_shutdown():
            t = rospy.Time.now().to_sec()
            self.cur_pos.z = self.oscillation_amplitude * math.sin(2 * math.pi * t * self.oscillation_rate)\
                             + self.oscillation_mean
            # Publish the position
            self.position_pub.publish(self.cur_pos)

            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('simple_movement_node')
    try:
        ktp = SimpleMovement()
    except rospy.ROSInterruptException:
        pass