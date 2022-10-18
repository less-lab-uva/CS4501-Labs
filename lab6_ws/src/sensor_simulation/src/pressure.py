#!/usr/bin/env python
import rospy
import time
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
import numpy as np

# Create a class which saves the altitude of the drone and esimates the pressure
class PressureSensor():

  # Node initialization
  def __init__(self):

    # Sleep to allow simulation to start
    time.sleep(5)

    # Create the publisher and subscriber
    self.pressure_pub = rospy.Publisher('/uav/sensors/pressure', Float64, queue_size=1)
    self.position_sub = rospy.Subscriber('/uav/sensors/ground_truth', PoseStamped, self.getPosition, queue_size=1)

    self.std_dev = float(rospy.get_param('/pressure_node/noise', str(10)))

    # Save the altitude of the drone
    self.altitude = 0

    # Used to estimate the pressure
    self.pressure = 0

    # Determines the baseline value of the pressure sensor at height 0m
    self.baseline_value = 0

    # Call the mainloop of our class
    self.mainloop()

  # Callback for the keyboard manager
  def getPosition(self, msg):
    # Save the drones alitude
    self.altitude = msg.pose.position.z

  # The main loop of the function
  def mainloop(self):
    # Set the rate of this loop
    rate = rospy.Rate(10)

    # Message for sending data
    pressure_msg = Float64()

    # While ROS is still running
    while not rospy.is_shutdown():

      # Publish the altitude
      self.pressure_pub.publish(pressure_msg)

      # Compute the pressure in milibars (according to https://www.weather.gov/media/epz/wxcalc/pressureAltitude.pdf)
      h = self.altitude
      common = pow((44307.7 - h),(12145.0/47571.0))
      self.pressure = (-3.86423)*(pow(10,-22))*(pow(h,5))*(common) + (8.56075)*(pow(10,-17))*(pow(h,4))*(common) - (7.58614)*(pow(10,-12))*(pow(h,3))*(common) + (3.36124)*(pow(10,-7))*(pow(h,2))*(common) - (0.00744645)*(h)*(common) + (65.987)*(common) 

      # Add sensor noise
      # noise = np.pow(10,-5) * random.uniform(-1, 1)
      noise = np.random.normal(0, self.std_dev)

      # Set the pressure
      pressure_msg.data = self.pressure + noise - self.baseline_value

      # Sleep for the remainder of the loop
      rate.sleep()


if __name__ == '__main__':
  rospy.init_node('pressure_simulation_node')
  try:
    ktp = PressureSensor()
  except rospy.ROSInterruptException:
    pass