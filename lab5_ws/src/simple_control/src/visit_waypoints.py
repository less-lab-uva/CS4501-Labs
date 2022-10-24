#!/usr/bin/env python
import rospy
import time
import math
import numpy as np
from geometry_msgs.msg import Vector3, PoseStamped


# A class used to visit different waypoints on a map
class VisitWaypoints():
  # On node initialization
  def __init__(self):
    # Init the waypoints [x,y] pairs, we will later obtain them from the command line
    waypoints_str = "[[2,6],[2,-4],[-2,-2],[4,-2],[-5,5],[4,4],[-6,-6]]"
    # Convert from string to numpy array
    try:
      self.waypoints=waypoints_str.replace('[','')
      self.waypoints=self.waypoints.replace(']','')
      self.waypoints=np.fromstring(self.waypoints,dtype=int,sep=',')
      self.waypoints=np.reshape(self.waypoints, (-1, 2))
    except:
      print("Error: Could not convert waypoints to Numpy array")
      print(waypoints_str)
      exit()

    # Create the position message we are going to be sending
    self.pos = Vector3()
    self.pos.z = 2.0

    # TODO Checkpoint 1:
    # Subscribe to `uav/sensors/gps`

    # Create the publisher and subscriber
    self.position_pub = rospy.Publisher('/uav/input/position_request', Vector3, queue_size=1)
    # Call the mainloop of our class
    self.mainloop()

  def mainloop(self):
    # Set the rate of this loop
    rate = rospy.Rate(1)

    # Wait 10 seconds before starting
    time.sleep(10)

    # TODO Checkpoint 1
    # Update mainloop() to fly between the different positions

    # While ROS is still running
    while not rospy.is_shutdown():

      # TODO Checkpoint 1
      # Check that we are at the waypoint for 5 seconds 
      
      # Publish the position
      self.position_pub.publish(self.pos)

      # Sleep for the remainder of the loop
      rate.sleep()


if __name__ == '__main__':
  rospy.init_node('visit_waypoints')
  try:
    ktp = VisitWaypoints()
  except rospy.ROSInterruptException:
    pass