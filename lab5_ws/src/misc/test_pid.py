#!/usr/bin/env python

PKG = 'simple_control'
NAME = 'test_pid'

import unittest, sys, time

import rospy, rostest

from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64


class TestPID(unittest.TestCase):
    
    def __init__(self, *args):
        super(TestPID, self).__init__(*args)       
        self.yaw_measurements = [float(s) for s  in  rospy.get_param('yaw').split(" ")]
        self.expected_controls = [float(s) for s in rospy.get_param('expected').split(" ")]
        self.actual_controls = []
        self.callback_received = False
        self.last_value = 0.0
                
    def callback(self, msg):
        if msg.data != 0.0:
            if self.last_value != msg.data:
                rospy.loginfo("Received new value " + str(msg.data))
                self.actual_controls.append(msg.data)
                self.last_value = msg.data
                self.callback_received = True

    def test_yaw_pid(self):
        rospy.init_node(NAME, anonymous=True)
        # subscribe to get the output value
        sub = rospy.Subscriber('/uav/input/yaw', Float64, self.callback)
        # create publisher
        pub = rospy.Publisher('/uav/sensors/attitude', Vector3, queue_size=10, latch=True)
        # wait until the yaw controller subscribes
        while pub.get_num_connections() == 0:
            rospy.loginfo("No subscribers, sleeping")
            rospy.sleep(0.1)

        for yaw in self.yaw_measurements:
            # publish measurement
            rospy.loginfo("test publishing"+ str(yaw))
            pub.publish(Vector3(0.0,0.0,yaw))
            # wait for a callback with a new value
            while not self.callback_received:
                rospy.loginfo("Waiting for callback")
                rospy.sleep(0.1)
            self.callback_received = False
            
        # check the results
        rospy.loginfo("Received " + str(len(self.actual_controls)) + "control values")
        for i in range(len(self.expected_controls)):
            self.assertTrue(abs(self.actual_controls[i] - self.expected_controls[i]) < 1e-9, "Expected value " + str(self.expected_controls[i]) + " received " + str(self.actual_controls[i]))
        
if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestPID, sys.argv)
