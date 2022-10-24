#!/usr/bin/env python
import rospy
import time
from keyboard.msg import Key
from geometry_msgs.msg import Vector3

# Create a class which we will use to take keyboard commands and convert them to a position
class KeyboardManager():
  # On node initialization
  def __init__(self):
    # Create the publisher and subscriber
    self.position_pub = rospy.Publisher('/uav/input/position', Vector3, queue_size=1)
    self.keyboard_sub = rospy.Subscriber('/keyboard/keydown', Key, self.get_key, queue_size = 1)
    # Create the position message we are going to be sending
    self.pos = Vector3()
    # Set the drones height to 3
    self.pos.z = 3
    # Create a variable we will use to hold the key code
    self.key_code = -1
    # Give the simulation enough time to start
    time.sleep(10)
    # Call the mainloop of our class
    self.mainloop()

  # Callback for the keyboard sub
  def get_key(self, msg):
    self.key_code = msg.code

  def mainloop(self):
    # Set the rate of this loop
    rate = rospy.Rate(20)

    # While ROS is still running
    while not rospy.is_shutdown():
      # Publish the position
      self.position_pub.publish(self.pos)

      # Left
      if self.key_code == Key.KEY_LEFT:
        self.pos.x += 1
      # Up
      if self.key_code == Key.KEY_UP:
        self.pos.y -= 1
      # Right
      if self.key_code == Key.KEY_RIGHT:
        self.pos.x -= 1
      # Down
      if self.key_code == Key.KEY_DOWN:
        self.pos.y += 1

      # Reset the code
      if self.key_code != -1:
        self.key_code = -1

    # Sleep for the remainder of the loop
    rate.sleep()

if __name__ == '__main__':
  rospy.init_node('keyboard_manager')
  try:
    ktp = KeyboardManager()
  except rospy.ROSInterruptException:
    pass
