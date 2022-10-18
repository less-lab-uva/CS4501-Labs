#!/usr/bin/env python
import rospy
import time
import math
import numpy as np
from threading import Lock

from geometry_msgs.msg import Vector3, PoseStamped, TwistStamped, Vector3Stamped
from std_msgs.msg import String, Bool, Float64
from sensor_msgs.msg import Image
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from velocity_pid_class import PID

# A class used to follow ship below drone
class ShipFollower():
  # On node initialization
  def __init__(self):

    # Allow the simulator to start
    time.sleep(3)

    # When this node shutsdown
    rospy.on_shutdown(self.shutdown_sequence)

    # TODO: Retrieve rate from ROS params

    self.dt = 1.0 / self.rate


    # TODO: Retrieve the PID parameters from ROS parameters
    

    # TODO: Initialize PIDs with PID class and ROS parameters


    # TODO: Initialize zero'ed class vars
    self.gps_height = 0

    # TODO: Create the publishers and subscribers

    # Run the control loop
    self.ControlLoop()

  # TODO FOR CHECKPOINT 1
  def get_gps(self, msg):
    # ONLY SAVE THE HEIGHT
    pass

  # TODO FOR CHECKPOINT 2
  # callback for /ship/beacon1
  def get_ship_beacon1(self, msg):
    pass

  # TODO FOR CHECKPOINT 2
  # callback for /ship/beacon2
  def get_ship_beacon2(self, msg):
    pass

  # TODO FOR CHECKPOINT 3
  # combine PID output from the two beacons
  # You don't have to use this method,
  # but you will need to use both error terms
  def combine_beacons(self):
    pass

  # This is the main loop of this class
  def ControlLoop(self):
    # Set the rate
    rate = rospy.Rate(self.rate)

    # While running
    while not rospy.is_shutdown():
      # Use PIDs to calculate the velocities you want
      # TODO FOR CHECKPOINT 1: z velocity

      # TODO FOR CHECKPOINT 2: x and y velocity
      # Sleep any excess time
      rate.sleep()

  # Called on ROS shutdown
  def shutdown_sequence(self):
    rospy.loginfo(str(rospy.get_name()) + ": Shutting Down")


if __name__ == '__main__':
  rospy.init_node('ship_follower_node')
  try:
    ktp = ShipFollower()
  except rospy.ROSInterruptException:
    pass
