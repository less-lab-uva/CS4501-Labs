#!/usr/bin/env python
import rospy
import cv2
from std_msgs.msg import Bool
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from marine_life_detection import MarineLifeDetection
from keras.preprocessing.image import img_to_array
from keras.models import load_model
from numpy import argmax
import os
import rospkg

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3' 

# Create a class which we will use to take keyboard commands and convert them to a position
class Perception():
  # On node initialization
  def __init__(self):
    # Allows us to read images from rostopics
    self.bridge = CvBridge()

    # Marine Life Detection node
    self.detector = MarineLifeDetection()
    self.detected = False
    self.classification = None
    self.image = None

    # Load the classes
    self.classes = ['dolphin', 'seal', 'penguin']

    # Load the model
    rp = rospkg.RosPack()
    model_path = os.path.join(rp.get_path("perception"), "machine_learning", "marine_model.h5")
    self.model = None
    if os.path.exists(model_path):
      self.model = load_model(model_path)

    # TODO Checkpoints 3 & 4
    # Create the publishers and subscriber
    self.image_sub = rospy.Subscriber("/uav/sensors/camera", Image, self.downfacing_camera_callback)

    # Call the mainloop of our class
    self.mainloop()

  def downfacing_camera_callback(self, msg):
    # TODO Checkpoint 3
    # Convert to opencv format using self.bridge
    # Use functions implemented in `marine_life_detection.py` to determine if animal is present
    # Save to self.detected

    # TODO Checkpoint 4
    # Save the opencv image to self.image so it can be processed further in the mainloop
    pass
    
  # Can be used to get image into the right structure to be passed into the model for CHECKPOINT 4
  # This function replaces `load_image` in the `classify_single_image` example
  def preprocess_image(self, img):
    # Reshape and convert to gray
    img = cv2.resize(img, (48, 48))
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img = img_to_array(gray)
    # reshape into a single sample with 1 channel
    img = img.reshape(1, 48, 48, 1)
    # prepare pixel data
    img = img.astype('float32')
    img = img / 255.0
    return img

  def mainloop(self):
    # Set the rate of this loop
    rate = rospy.Rate(0.25)

    # While ROS is still running
    while not rospy.is_shutdown():

      # TODO Checkpoint 3
      # Publish wether an animal was detected on /marine_life_detected

      if self.model is not None:
        if self.detected:
          # TODO Checkpoint 4
          # If the animal was detected
          # Load the image (using self.preprocess_image)
          # Predict what class it is
          # Get the class index using argmax
          # Save the classification to self.classification
          pass

        if self.classification is not None:
          # TODO CHeckpoint 4
          # Publish the class on /marine_life_classification
          pass

      # Sleep for the remainder of the loop
      rate.sleep()


if __name__ == '__main__':
  rospy.init_node('PerceptionNode')
  try:
    ktp = Perception()
  except rospy.ROSInterruptException:
    pass