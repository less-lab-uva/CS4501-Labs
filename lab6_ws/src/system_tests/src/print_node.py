#!/usr/bin/env python
import rospy
from std_msgs.msg import String

# Create a class which we will use to take keyboard commands and convert them to a position
class PrintNode():
  def __init__(self):

    self.image_sub = rospy.Subscriber("/test_debug", String, self.debug_log_callback)

    # Call the mainloop of our class
    self.mainloop()

  def debug_log_callback(self, msg):
    rospy.loginfo(msg.data)
    
  def mainloop(self):
    # Set the rate of this loop
    rate = rospy.Rate(30)

    # While ROS is still running
    while not rospy.is_shutdown():

      # Sleep for the remainder of the loop
      rate.sleep()

if __name__ == '__main__':
  rospy.init_node('PrintNode')
  try:
    pn = PrintNode()
  except rospy.ROSInterruptException:
    pass