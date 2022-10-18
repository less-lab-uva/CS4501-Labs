#!/usr/bin/env python

import sys
import time
import math
import unittest
import numpy as np

import rospy
import rostest

from std_msgs.msg import String, Bool
from geometry_msgs.msg import Vector3, PoseStamped

class TestDroneBehavior(unittest.TestCase):

    def __init__(self, *args):
        super(TestDroneBehavior, self).__init__(*args)
        rospy.init_node("test_behavior", anonymous=True)
        # Publish the debug information
        self.debug_logger = rospy.Publisher('/test_debug', String, queue_size=1)
        # Get the test duration
        self.test_duration = rospy.get_param(rospy.get_name() + '/duration')

    # Print the message
    def print_msg(self, incoming_data):
        msg = String()
        msg.data = str(incoming_data)
        self.debug_logger.publish(msg)


    # TODO: Update this function to check that once a ship is detected, it is never lost.
    def test_following(self):
        self.print_msg("Starting: test_following")
        # Get the start time
        start_time = time.time()
        # Test for a set duration
        while time.time() - start_time < self.test_duration:
            detected = rospy.wait_for_message("/ship/followed", Bool, timeout=None)
            # Hint: use self.assertEqual()

            rospy.sleep(0.1)

if __name__ == '__main__':
    rostest.rosrun("system_tests", "drone_behavior_test", TestDroneBehavior, sys.argv)
