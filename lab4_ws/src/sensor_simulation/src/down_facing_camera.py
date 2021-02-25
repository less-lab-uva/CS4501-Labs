#!/usr/bin/env python
import rospy
import cv2
import math
import os
import copy
import numpy as np
from tf.transformations import euler_from_quaternion

from cv_bridge import CvBridge, CvBridgeError 
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped


# Create a class which saves the altitude of the drone and esimates the pressure
class DownFacingCamera():

  # Node initialization
  def __init__(self):

    # Get the default path (need to remove src)
    self.dir_path = os.path.dirname(os.path.realpath(__file__))
    self.dir_path = self.dir_path[:-3] + "data/"

    # Get image name and load it from data folder
    self.img_path = rospy.get_param('/down_facing_camera_node/image_path', "constructon.jpeg")
    self.full_image = cv2.imread(self.dir_path + self.img_path)

    # Check if an image was loaded
    if self.full_image is None:
      rospy.logerr(str(rospy.get_name()) + ": Could not load image - " + str(self.dir_path + self.img_path) + " - closing node.")
      exit()

    # Create the cv bridge
    self.bridge = CvBridge()

    # Create the subscribers
    self.position_sub = rospy.Subscriber('/uav/sensors/ground_truth', PoseStamped, self.getPosition, queue_size = 1)

    # Create the publishers 
    self.image_pub = rospy.Publisher("/uav/sensors/camera", Image, queue_size=10)
    
    # Save the drones positions
    self.position = np.zeros(3, dtype=np.float64)
    self.yaw = 0

    # Call the mainloop of our class
    self.mainloop()


  # Callback for the keyboard manager
  def getPosition(self, msg):
    # Save the drones alitude
    self.position[0] = msg.pose.position.x
    self.position[1] = msg.pose.position.y
    self.position[2] = msg.pose.position.z

    # Get the drones attitude
    quarternion_pose = (msg.pose.orientation.x,
                        msg.pose.orientation.y,
                        msg.pose.orientation.z,
                        msg.pose.orientation.w)
    self.yaw = np.array(euler_from_quaternion(quarternion_pose))[2]


  # The main loop of the function
  def mainloop(self):
    # Set the rate of this loop
    rate = rospy.Rate(10)

    # Used to only display the warning once
    warning_state = False

    # While ROS is still running
    while not rospy.is_shutdown():

      try:

        # Create a copy of the image
        img = copy.deepcopy(self.full_image)

        # Get the center of the image
        img_center = (img.shape[0] / 2.0, img.shape[1] / 2.0)

        # TODO Can we map the image shape to a map size to autocompute the coefficient
        # Compute the area of the drones image
        height_coefficient = 150 # 150 units per 1m up
        camera_fov = math.radians(30)
        drone_leg_height = 0.05
        img_length = (((self.position[2] + drone_leg_height) * height_coefficient) * math.tan(camera_fov)) * 2
        img_area = img_length * img_length

        # Compute the camera center
        move_coefficent = 150 # 150 per 1m unit moved
        camera_center = (img_center[1] + (self.position[1] * move_coefficent), img_center[0] + (self.position[0]* move_coefficent))

        # Rotate the original image around the camera center
        # calculate the rotation matrix
        M = cv2.getRotationMatrix2D(camera_center, math.degrees(self.yaw), 1)
        # rotate the original image
        rot_img = cv2.warpAffine(img, M, (img.shape[1], img.shape[0]))

        # Grab the image
        start_point = (int(camera_center[0] - (img_length / 2.0)), int(camera_center[1] - (img_length / 2.0)))
        end_point = (int(camera_center[0] + (img_length / 2.0)), int(camera_center[1] + (img_length / 2.0)))

        # partial_image = cv2.rectangle(rot_img, start_point, end_point, (0,255,0), 3) 
        partial_image = rot_img[start_point[1]:end_point[1], start_point[0]:end_point[0], 0:3]

        if (partial_image.shape[0] <= 0 or partial_image.shape[1] <= 0):
          if not warning_state:
            rospy.logwarn(str(rospy.get_name()) + ": One of the image dimensions is 0.")
            warning_state = True
          continue

        # We are not in a warning state anymore
        warning_state = False

        # Resize 
        partial_image = cv2.resize(partial_image, (200, 200))

        # Publish
        img_msg = self.bridge.cv2_to_imgmsg(partial_image, "bgr8")
        self.image_pub.publish(img_msg)

      # Print error
      except CvBridgeError, e:
        print e

      # Sleep for the remainder of the loop
      rate.sleep()


if __name__ == '__main__':
  rospy.init_node('down_facing_camera_node')
  try:
    ktp = DownFacingCamera()
  except rospy.ROSInterruptException:
    pass