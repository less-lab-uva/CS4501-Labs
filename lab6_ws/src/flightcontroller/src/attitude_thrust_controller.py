#!/usr/bin/env python
import rospy
import time
import math
import numpy as np

from pid_class import PID
from geometry_msgs.msg import TwistStamped, Vector3
from std_msgs.msg import Float64

class AttitudeThrustController():

  def __init__(self):

    # Allow the simulator to start
    time.sleep(5)

    # When this node shutsdown
    rospy.on_shutdown(self.shutdown_sequence)

    # Set the rate
    self.rate = 100.0
    self.dt = 1.0 / self.rate

    # Create the setpoints
    self.setpoints = np.zeros(3, dtype=np.float64)

    # Create the current output readings
    # self.vel_readings =np.zeros(3, dtype=np.float64)

    # Initialize the yaw reading
    self.yaw_reading = 0
    
    # Create the subscribers
    # self.vel_read_sub = rospy.Subscriber("/uav/sensors/velocity", TwistStamped, self.get_vel)
    self.vel_set_sub = rospy.Subscriber('/uav/input/velocityPID', Vector3 , self.set_vel)
    self.sub = rospy.Subscriber('/uav/sensors/attitude', Vector3, self.euler_angle_callback)

    # Create the publishers 
    self.att_pub = rospy.Publisher("/uav/input/attitude", Vector3, queue_size=1)
    self.thrust_pub = rospy.Publisher('/uav/input/thrust', Float64, queue_size=1)

    # Run the control loop
    self.ControlLoop()

  # This is the main loop of this class
  def ControlLoop(self):
    # Set the rate
    rate = rospy.Rate(10) # 50

    # Keep track how many loops have happend
    loop_counter = 0

    # Data we will be publishing
    z_output = Float64()
    z_output.data = 0

    # While running
    while not rospy.is_shutdown():

      # Use a PID to calculate the angle you want to hold and thrust you want
      output = self.setpoints
      # rospy.loginfo("self.setpoints" + str(self.setpoints))
      # Transform X and Y velocity from world frame to drone frame (i.e. yaw needs to be taken into account)
      x_output =   math.cos(self.yaw_reading) * output[1] - math.sin(self.yaw_reading) * output[0]
      y_output = - math.sin(self.yaw_reading) * output[1] - math.cos(self.yaw_reading) * output[0]

      # Set the yaw at 0
      yaw = 0

      # The z output is the PID + gravity
      z_output.data = output[2] + 9.8

      # Limit the thrust to the drone
      z_output.data = min(max(0, z_output.data), 20)

      # Create and publish the data (0 yaw)
      attitude = Vector3(x_output, y_output, yaw)
      self.att_pub.publish(attitude) 
      self.thrust_pub.publish(z_output)

      # Sleep any excess time
      rate.sleep()


  # Call back to get the velocity data
  # def get_vel(self, vel_msg):
  #   self.vel_readings[0] = vel_msg.twist.linear.x
  #   self.vel_readings[1] = vel_msg.twist.linear.y
  #   self.vel_readings[2] = vel_msg.twist.linear.z

  # Get the yaw reading
  def euler_angle_callback(self, msg):
    self.yaw_reading = msg.z

  # Call back to get the velocity setpoints
  def set_vel(self, vel_msg):
    self.setpoints[0] = vel_msg.x
    self.setpoints[1] = vel_msg.y
    self.setpoints[2] = vel_msg.z

	# Called on ROS shutdown
  def shutdown_sequence(self):
    rospy.loginfo(str(rospy.get_name()) + ": Shutting Down")


def main():
  rospy.init_node('attitude_thrust_controller_node')
  try:
    velcon = AttitudeThrustController()
  except rospy.ROSInterruptException:
    pass


if __name__ == '__main__':
  main()