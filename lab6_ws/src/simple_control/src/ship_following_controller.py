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
    self.rate = rospy.get_param('/ship_following_controller_node/rate')

    self.dt = 1.0 / self.rate

    # TODO: Retrieve the PID parameters from ROS parameters
    self.pxy = rospy.get_param('/ship_following_controller_node/gains/p_xy')
    self.ixy = rospy.get_param('/ship_following_controller_node/gains/i_xy')
    self.dxy = rospy.get_param('/ship_following_controller_node/gains/d_xy')

    self.pz = rospy.get_param('/ship_following_controller_node/gains/p_z')
    self.iz = rospy.get_param('/ship_following_controller_node/gains/i_z')
    self.dz = rospy.get_param('/ship_following_controller_node/gains/d_z')

    # TODO: Initialize PIDs with PID class and ROS parameters
    self.pid_x = PID(self.pxy, self.ixy, self.dxy)
    self.pid_y = PID(self.pxy, self.ixy, self.dxy)
    self.pid_z = PID(self.pz, self.iz, self.dz)

    # TODO: Initialize zero'ed class vars
    self.gps_height = 0
    self.x_vel = 0
    self.y_vel = 0
    self.z_vel = 0
    self.velocity = Vector3()
    self.beacon1 = None
    self.beacon2 = None


    # TODO: Create the publishers and subscribers
    self.beacon2_sub = rospy.Subscriber('/ship/beacon1',
                                         Vector3Stamped, self.get_ship_beacon1,
                                         queue_size=1)
    self.beacon2_sub = rospy.Subscriber('/ship/beacon2',
                                        Vector3Stamped, self.get_ship_beacon2,
                                        queue_size=1)
    self.gps_sub = rospy.Subscriber('/uav/sensors/gps',
                                        PoseStamped, self.get_gps,
                                        queue_size=1)


    self.velocity_pub = rospy.Publisher('/uav/input/velocityPID', Vector3, queue_size=1)

    self.estim_pub = rospy.Publisher('/ship/estimated_position', Vector3, queue_size=1)

    # Run the control loop
    self.ControlLoop()

  # TODO FOR CHECKPOINT 1
  def get_gps(self, msg):
    # ONLY SAVE THE HEIGHT
    self.gps_height = msg.pose.position.z

  # TODO FOR CHECKPOINT 2
  # callback for /ship/beacon1
  def get_ship_beacon1(self, msg):
    self.beacon1 = msg

  # TODO FOR CHECKPOINT 2
  # callback for /ship/beacon2
  def get_ship_beacon2(self, msg):
    self.beacon2 = msg

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
    loop_counter = 0

    # While running
    while not rospy.is_shutdown():
      # Use PIDs to calculate the velocities you want
      # TODO FOR CHECKPOINT 1: z velocity
      self.z_vel = self.pid_z.pid_loop(10 - self.gps_height, self.rate)

      # TODO FOR CHECKPOINT 2: x and y velocity
      error_x = 0
      error_y = 0
      if self.beacon1 is not None:
        error_x = self.beacon1.vector.x
        error_y = self.beacon1.vector.y
      # do not use beacon1 is beacon 2 is publishing
      if self.beacon2 is not None:
        error_x = (self.beacon2.vector.x)
        error_y = (self.beacon2.vector.y)

      self.x_vel = self.pid_x.pid_loop(error_x, self.rate)
      self.y_vel = self.pid_y.pid_loop(error_y, self.rate)


      # fill velocity message and publish

      self.velocity.x = self.x_vel #if abs(self.x_vel) > 1e-3 else 0
      self.velocity.y = self.y_vel #if abs(self.y_vel) > 1e-3 else 0
      self.velocity.z = self.z_vel

      self.velocity_pub.publish(self.velocity)
      self.estim_pub.publish()

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
