#!/usr/bin/env python
import cv2
import numpy as np

# Create a class which we will use to take keyboard commands and convert them to a position
class MarineLifeDetection():
  
  def create_marine_life_mask(self, image):
    # Convert the image to the HSV format using opencv
    # Define your HSV upper values
    # Define your HSV lower values
    # Generate a mask using opencv inrange function.
    # Return the mask
    return None

  def segment_image(self, image, mask):
    # Return the bitwise_and of the image and the mask. Note opencv has a bitwise_and operation
    # Return the resultant image
    return None

  def marine_life_detected(self, mask):
    # Compute the size of the mask
    # Compute the number of marine animal pixels in the mask
    # Use the ratio of these values to determine whether an animal was detected or not
    # Return whether an animal is detected or not
    return False